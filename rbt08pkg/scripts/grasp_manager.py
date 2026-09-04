#!/usr/bin/env python3
"""Coordinate the detachable joints used by the pear-harvesting scene."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from ros_gz_interfaces.srv import ControlWorld
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String


class GraspManager(Node):
    """Attach a pear only after a two-finger grasp, then release it safely."""

    CONTACT_TIMEOUT_S = 0.25
    CLOSED_POSITION_M = 0.050
    OPEN_POSITION_M = 0.010

    def __init__(self) -> None:
        super().__init__('grasp_manager')
        self._state = 'INITIALIZING'
        self._left_contact_at: float | None = None
        self._right_contact_at: float | None = None
        self._finger_positions = {
            'left_finger_joint': 0.0,
            'right_finger_joint': 0.0,
        }
        self._world_started = False

        self._tree_attach = self.create_publisher(Empty, '/harvest/tree/attach', 10)
        self._tree_detach = self.create_publisher(Empty, '/harvest/tree/detach', 10)
        self._gripper_attach = self.create_publisher(Empty, '/harvest/gripper/attach', 10)
        self._gripper_detach = self.create_publisher(Empty, '/harvest/gripper/detach', 10)
        self._status = self.create_publisher(String, '/harvest/state', 10)

        self.create_subscription(String, '/harvest/tree/state', self._on_tree_state, 10)
        self.create_subscription(String, '/harvest/gripper/state', self._on_gripper_state, 10)
        self.create_subscription(Contacts, '/harvest/contacts/left', self._on_left_contacts, 10)
        self.create_subscription(Contacts, '/harvest/contacts/right', self._on_right_contacts, 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 20)

        self._world_control = self.create_client(
            ControlWorld, '/world/pear_orchard/control')
        self.create_timer(0.2, self._tick)
        self._publish_status('waiting for the tree-to-pear joint')

    def _publish_status(self, text: str) -> None:
        self._status.publish(String(data=f'{self._state}: {text}'))
        self.get_logger().info(text)

    @staticmethod
    def _touches_pear(msg: Contacts) -> bool:
        for contact in msg.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            if any('pear_' in name or name == 'pear' for name in names):
                return True
        return False

    def _on_left_contacts(self, msg: Contacts) -> None:
        if self._touches_pear(msg):
            self._left_contact_at = time.monotonic()

    def _on_right_contacts(self, msg: Contacts) -> None:
        if self._touches_pear(msg):
            self._right_contact_at = time.monotonic()

    def _on_joint_state(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name in self._finger_positions:
                self._finger_positions[name] = position

    def _on_tree_state(self, msg: String) -> None:
        if msg.data != 'attached' or self._state != 'INITIALIZING':
            return
        self._state = 'ON_TREE'
        self._publish_status('pear attached to the tree; starting the world')
        self._start_world()

    def _on_gripper_state(self, msg: String) -> None:
        if msg.data == 'attached' and self._state == 'ATTACHING_GRIPPER':
            self._tree_detach.publish(Empty())
            self._state = 'ON_GRIPPER'
            self._publish_status('pear attached to gripper; tree joint detached')
        elif msg.data == 'detached' and self._state == 'RELEASING':
            self._state = 'RELEASED'
            self._publish_status('pear released and governed by physics')

    def _start_world(self) -> None:
        if self._world_started:
            return
        if not self._world_control.service_is_ready():
            self.get_logger().warning('world-control service is not ready; retrying')
            return
        request = ControlWorld.Request()
        request.world_control.pause = False
        future = self._world_control.call_async(request)
        future.add_done_callback(self._on_world_started)
        self._world_started = True

    def _on_world_started(self, future) -> None:
        try:
            if not future.result().success:
                self.get_logger().error('Gazebo rejected the request to start the world')
        except Exception as error:  # Service transport errors should not crash grasping.
            self.get_logger().error(f'Unable to start the world: {error}')

    def _recent_two_finger_contact(self) -> bool:
        now = time.monotonic()
        return (
            self._left_contact_at is not None
            and self._right_contact_at is not None
            and now - self._left_contact_at <= self.CONTACT_TIMEOUT_S
            and now - self._right_contact_at <= self.CONTACT_TIMEOUT_S
        )

    def _gripper_is_closed(self) -> bool:
        return all(position >= self.CLOSED_POSITION_M
                   for position in self._finger_positions.values())

    def _gripper_is_open(self) -> bool:
        return all(position <= self.OPEN_POSITION_M
                   for position in self._finger_positions.values())

    def _tick(self) -> None:
        if self._state == 'INITIALIZING':
            # The world is launched paused, so this can safely be retried.
            self._tree_attach.publish(Empty())
            return

        if self._state == 'ON_TREE':
            if not self._world_started:
                self._start_world()
            if self._recent_two_finger_contact() and self._gripper_is_closed():
                self._gripper_attach.publish(Empty())
                self._state = 'ATTACHING_GRIPPER'
                self._publish_status('two-finger grasp confirmed; attaching pear to gripper')
            return

        if self._state == 'ON_GRIPPER' and self._gripper_is_open():
            self._gripper_detach.publish(Empty())
            self._state = 'RELEASING'
            self._publish_status('gripper opened; releasing pear')


def main() -> None:
    rclpy.init()
    node = GraspManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

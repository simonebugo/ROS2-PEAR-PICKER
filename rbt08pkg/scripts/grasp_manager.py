#!/usr/bin/env python3
"""Coordinate the detachable joints used by the pear-harvesting scene."""

from __future__ import annotations

import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from ros_gz_interfaces.srv import ControlWorld
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String


class GraspManager(Node):
    """Attach a pear only after a two-finger grasp, then release it safely."""

    CONTACT_TIMEOUT_S = 0.25
    # With the current geometry fingertip contact starts around 12 mm of
    # inward travel. We accept the grasp just before that nominal value and
    # reserve a much smaller threshold for a genuinely open gripper.
    CLOSED_POSITION_M = 0.008
    OPEN_POSITION_M = 0.003

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
        self._step_pending = False
        self._tree_attached_at: float | None = None

        self._tree_attach = self.create_publisher(
            Empty, '/harvest/tree/attach', 10)
        self._tree_detach = self.create_publisher(
            Empty, '/harvest/tree/detach', 10)
        self._gripper_attach = self.create_publisher(
            Empty, '/harvest/gripper/attach', 10)
        self._gripper_detach = self.create_publisher(
            Empty, '/harvest/gripper/detach', 10)
        self._status = self.create_publisher(String, '/harvest/state', 10)

        self.create_subscription(
            String, '/harvest/tree/state', self._on_tree_state, 10)
        self.create_subscription(
            String, '/harvest/gripper/state', self._on_gripper_state, 10)
        self.create_subscription(
            Contacts, '/harvest/contacts/left', self._on_left_contacts, 10)
        self.create_subscription(
            Contacts, '/harvest/contacts/right', self._on_right_contacts, 10)
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 20)

        self._world_control = self.create_client(
            ControlWorld, '/world/pear_orchard/control')
        # This timer must run even while simulation time is paused: its first
        # job is attaching the pear to the tree before unpausing Gazebo.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.2, self._tick, clock=self._steady_clock)
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
        if msg.data == 'attached' and self._state == 'INITIALIZING':
            self._state = 'ON_TREE'
            # The attached event is emitted from Gazebo PreUpdate, so it is
            # proof that the one paused step has been processed.  The ROS
            # response for that service call can remain pending while the
            # world is paused; do not use that response as a prerequisite for
            # starting the simulation.
            self._tree_attached_at = time.monotonic()
            self._publish_status('pear attached to the tree; ready to start the world')
        elif msg.data == 'detached' and self._state == 'DETACHING_TREE':
            # Never attach the pear to tree and robot simultaneously: that
            # would create a closed kinematic loop, unsupported by Gazebo's
            # DetachableJoint system.
            self._gripper_attach.publish(Empty())
            self._state = 'ATTACHING_GRIPPER'
            self._publish_status('tree joint detached; attaching pear to gripper')

    def _on_gripper_state(self, msg: String) -> None:
        if msg.data == 'attached' and self._state == 'ATTACHING_GRIPPER':
            self._state = 'ON_GRIPPER'
            self._publish_status('pear attached to gripper')
        elif msg.data == 'detached' and self._state == 'RELEASING':
            self._state = 'RELEASED'
            self._publish_status('pear released and governed by physics')

    def _start_world(self) -> None:
        if self._world_started:
            return
        if not self._world_control.service_is_ready():
            self.get_logger().warning(
                'world-control service is not ready; retrying')
            return
        request = ControlWorld.Request()
        request.world_control.pause = False
        self.get_logger().info('requesting Gazebo to start the world')
        future = self._world_control.call_async(request)
        future.add_done_callback(self._on_world_started)
        self._world_started = True

    def _step_world_once(self) -> None:
        """Advance a paused world so Gazebo can process the initial attach."""
        if self._step_pending or not self._world_control.service_is_ready():
            return
        request = ControlWorld.Request()
        request.world_control.pause = True
        request.world_control.step = True
        future = self._world_control.call_async(request)
        future.add_done_callback(self._on_world_step)
        self._step_pending = True

    def _on_world_step(self, future) -> None:
        self._step_pending = False
        try:
            if not future.result().success:
                self.get_logger().warning(
                    'Gazebo rejected an initialization step')
        except Exception as error:
            self.get_logger().warning(
                f'Unable to step Gazebo during initialization: {error}')

    def _on_world_started(self, future) -> None:
        try:
            if not future.result().success:
                self._world_started = False
                self.get_logger().error(
                    'Gazebo rejected the request to start the world')
        except Exception as error:
            self._world_started = False
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
        return all(
            position >= self.CLOSED_POSITION_M
            for position in self._finger_positions.values()
        )

    def _gripper_is_open(self) -> bool:
        return all(
            position <= self.OPEN_POSITION_M
            for position in self._finger_positions.values()
        )

    def _tick(self) -> None:
        if self._state == 'INITIALIZING':
            # DetachableJoint handles requests in PreUpdate. Advance one
            # iteration while paused so it can create the initial tree joint.
            self._tree_attach.publish(Empty())
            self._step_world_once()
            return

        if self._state == 'ON_TREE':
            # Keep a small real-time gap after the attach event: it ensures
            # the preceding pause=true/step=true request has been consumed
            # before pause=false is queued, without waiting on its response.
            tree_attached_for = (
                time.monotonic() - self._tree_attached_at
                if self._tree_attached_at is not None else 0.0)
            if not self._world_started and tree_attached_for >= 0.2:
                self._start_world()
            if self._recent_two_finger_contact() and self._gripper_is_closed():
                self._tree_detach.publish(Empty())
                self._state = 'DETACHING_TREE'
                self._publish_status(
                    'two-finger grasp confirmed; detaching pear from tree')
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

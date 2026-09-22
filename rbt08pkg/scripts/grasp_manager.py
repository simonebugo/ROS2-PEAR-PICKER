#!/usr/bin/env python3
"""Coordinate the detachable joints used by the pear-harvesting scene."""

from __future__ import annotations

import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String


class GraspManager(Node):
    """Transfer the pear from tree to gripper after contact or close confirmation."""

    # Contact messages from the two independent sensors do not necessarily
    # arrive in the same Gazebo update, especially on a loaded VM.
    CONTACT_TIMEOUT_S = 1.0
    # Release once both fingers have returned near the open position.
    OPEN_POSITION_M = 0.003

    def __init__(self) -> None:
        super().__init__('grasp_manager')
        # Gazebo's DetachableJoint starts attached by default.  The gripper
        # joint is therefore explicitly detached before the tree joint is
        # initialized; otherwise the pear keeps the large robot-to-fruit
        # transform that exists when the robot is spawned.
        self._state = 'INITIALIZING_GRIPPER_DETACH'
        self._status_text = 'starting'
        self._left_contact_at: float | None = None
        self._right_contact_at: float | None = None
        self._close_command_complete = False
        self._tree_joint_state: str | None = None
        self._gripper_joint_state: str | None = None
        self._finger_positions = {
            'left_finger_joint': 0.0,
            'right_finger_joint': 0.0,
        }
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
        self.create_subscription(
            Empty,
            '/harvest/gripper/close_complete',
            self._on_close_command_complete,
            10,
        )

        # Keep initialization independent from simulation time while Gazebo
        # and the clock bridge are still starting.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.2, self._tick, clock=self._steady_clock)
        self.create_timer(
            1.0, self._republish_status, clock=self._steady_clock)
        self._publish_status(
            'detaching the default robot-to-pear joint before initialization')

    def _publish_status(self, text: str) -> None:
        self._status_text = text
        self._republish_status()
        self.get_logger().info(text)

    def _republish_status(self) -> None:
        self._status.publish(
            String(data=f'{self._state}: {self._status_text}'))

    @staticmethod
    def _touches_pear(msg: Contacts) -> bool:
        for contact in msg.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            if any('pear_' in name or name == 'pear' for name in names):
                return True
        return False

    def _on_left_contacts(self, msg: Contacts) -> None:
        if self._touches_pear(msg):
            if self._left_contact_at is None:
                self.get_logger().info('left finger contact with pear detected')
            self._left_contact_at = time.monotonic()

    def _on_right_contacts(self, msg: Contacts) -> None:
        if self._touches_pear(msg):
            if self._right_contact_at is None:
                self.get_logger().info('right finger contact with pear detected')
            self._right_contact_at = time.monotonic()

    def _on_joint_state(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name in self._finger_positions:
                self._finger_positions[name] = position

    def _on_close_command_complete(self, _msg: Empty) -> None:
        """Record that the controller completed the deliberate grasp command.

        Gazebo may stop the prismatic joints against the fruit without
        publishing ContactSensor samples.  The completion signal is emitted
        only after the robot has reached the known grasp pose and the gripper
        controller has finished its close trajectory.
        """
        if self._state == 'ON_TREE':
            self._close_command_complete = True
            positions = ', '.join(
                f'{name}={position:.4f}'
                for name, position in self._finger_positions.items()
            )
            self.get_logger().info(
                f'gripper close command completed ({positions})')

    def _on_tree_state(self, msg: String) -> None:
        self._tree_joint_state = msg.data
        if msg.data == 'attached' and self._state == 'INITIALIZING_TREE_ATTACH':
            self._state = 'ON_TREE'
            self._publish_status('pear attached to the tree; world running')
        elif msg.data == 'detached' and self._state == 'DETACHING_TREE':
            # Never attach the pear to tree and robot simultaneously: that
            # would create a closed kinematic loop, unsupported by Gazebo's
            # DetachableJoint system.
            self._state = 'ATTACHING_GRIPPER'
            self._gripper_attach.publish(Empty())
            self._publish_status('tree joint detached; attaching pear to gripper')

    def _on_gripper_state(self, msg: String) -> None:
        self._gripper_joint_state = msg.data
        if (msg.data == 'detached' and
                self._state == 'INITIALIZING_GRIPPER_DETACH'):
            # Only now is it safe to let the tree joint own the pear.  This
            # prevents two simultaneous fixed joints and the resulting
            # kinematic loop / spawn-time grasp offset.
            if self._tree_joint_state == 'attached':
                # The tree plugin may have reported its default attached state
                # while the gripper joint was still being initialized.
                self._state = 'ON_TREE'
                self._publish_status(
                    'robot-to-pear joint detached; pear remains on tree')
            else:
                self._state = 'INITIALIZING_TREE_ATTACH'
                self._tree_attach.publish(Empty())
                self._publish_status(
                    'robot-to-pear joint detached; attaching pear to tree')
        elif msg.data == 'attached' and self._state == 'ATTACHING_GRIPPER':
            self._state = 'ON_GRIPPER'
            self._publish_status('pear attached to gripper')
        elif msg.data == 'detached' and self._state == 'RELEASING':
            self._state = 'RELEASED'
            self._publish_status('pear released and governed by physics')

    def _recent_two_finger_contact(self) -> bool:
        now = time.monotonic()
        return (
            self._left_contact_at is not None
            and self._right_contact_at is not None
            and now - self._left_contact_at <= self.CONTACT_TIMEOUT_S
            and now - self._right_contact_at <= self.CONTACT_TIMEOUT_S
        )

    def _gripper_is_open(self) -> bool:
        return all(
            position <= self.OPEN_POSITION_M
            for position in self._finger_positions.values()
        )

    def _tick(self) -> None:
        if self._state == 'INITIALIZING_GRIPPER_DETACH':
            # DetachableJoint starts attached. Repeat the command because the
            # ROS-Gazebo bridge and the robot plugin may still be starting.
            self._gripper_detach.publish(Empty())
            return

        if self._state == 'INITIALIZING_TREE_ATTACH':
            # Gazebo is launched with -r, so PreUpdate processes this request
            # without pausing or stepping the world from ROS.
            self._tree_attach.publish(Empty())
            return

        if self._state == 'ON_TREE':
            contacts_confirmed = self._recent_two_finger_contact()
            if contacts_confirmed or self._close_command_complete:
                self._tree_detach.publish(Empty())
                self._state = 'DETACHING_TREE'
                confirmation = (
                    'two-finger contact confirmed'
                    if contacts_confirmed
                    else 'completed gripper closure confirmed'
                )
                self._publish_status(
                    f'{confirmation}; detaching pear from tree')
            return

        if self._state == 'ATTACHING_GRIPPER':
            # Empty commands are best-effort across the ROS-Gazebo bridge.
            # Repeat until Gazebo publishes the real "attached" state. Never
            # infer success from elapsed time: doing so previously hid the
            # fact that the joint had already been attached since startup.
            self._gripper_attach.publish(Empty())
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

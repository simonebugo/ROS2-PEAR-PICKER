#!/usr/bin/env python3

import sys

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from rclpy.action import ActionClient
from rclpy.node import Node


class MoveItSmokeTest(Node):
    """Plan and execute one small arm motion through MoveIt."""

    TARGETS = {
        'j1': 0.20,
        'j2': -0.15,
        'j3': 0.15,
        'j4': 0.0,
        'j5': 0.0,
        'j6': 0.0,
    }

    def __init__(self):
        super().__init__('moveit_smoke_test')
        self._client = ActionClient(self, MoveGroup, '/move_action')

    def run(self) -> bool:
        self.get_logger().info('Attendo il server MoveIt /move_action...')
        if not self._client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('Server /move_action non disponibile.')
            return False

        constraints = Constraints()
        constraints.name = 'gazebo_smoke_test_target'
        for joint_name, position in self.TARGETS.items():
            constraint = JointConstraint()
            constraint.joint_name = joint_name
            constraint.position = position
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)

        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.10
        goal.request.max_acceleration_scaling_factor = 0.10
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False

        self.get_logger().info(
            'Invio a MoveIt un singolo goal articolare per il robot Gazebo...'
        )
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done():
            self.get_logger().error('Timeout durante l’invio del goal a MoveIt.')
            return False
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Goal rifiutato da MoveIt.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        if not result_future.done():
            self.get_logger().error('Timeout durante pianificazione o esecuzione.')
            return False
        result = result_future.result()
        if result is None:
            self.get_logger().error('Timeout durante pianificazione o esecuzione.')
            return False

        error_code = result.result.error_code.val
        if error_code != 1:
            self.get_logger().error(
                f'MoveIt ha terminato con codice errore {error_code}.'
            )
            return False

        self.get_logger().info(
            'TEST SUPERATO: traiettoria pianificata da MoveIt ed eseguita in Gazebo.'
        )
        return True


def main(args=None):
    rclpy.init(args=args)
    node = MoveItSmokeTest()
    success = False
    try:
        success = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())

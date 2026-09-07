#!/usr/bin/env python3

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class PearPoseSource(Node):
    def __init__(self):
        super().__init__('pear_pose_source')
        self.publisher = self.create_publisher(PoseStamped, '/harvest/pear_pose', 10)
        # The known pose must also be available while Gazebo is paused during
        # initialization, so this publisher must not depend on /clock.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            0.5, self.publish_pear_pose, clock=self._steady_clock)  # 2 Hz
        self.get_logger().info('PearPoseSource node started. Publishing static pear pose to /harvest/pear_pose at 2 Hz...')

    def publish_pear_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        # Posa statica coerente con la pera sospesa nel world SDF.
        msg.pose.position.x = 0.85
        msg.pose.position.y = 0.55
        msg.pose.position.z = 1.30

        # Conventional orientation (Identity quaternion)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PearPoseSource()
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

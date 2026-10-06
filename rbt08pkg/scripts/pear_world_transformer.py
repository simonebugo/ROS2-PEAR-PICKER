#!/usr/bin/env python3
"""Transform observed surface points at acquisition time; never command the robot."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker
from pear_center import estimate_center


class PearWorldTransformer(Node):
    def __init__(self):
        super().__init__('pear_world_transformer')
        for name, value in {
            'source_frame': 'table_camera_link_optical', 'target_frame': 'world',
            'point_topic': '/orchard/vision/surface_point',
            'source_marker_topic': '/orchard/vision/surface_marker',
            'stream_timeout_s': 10.0,
            'estimate_center': True, 'fruit_radius_m': 0.075,
            'publish_harvest_pose': False, 'pear_yaw_world': 0.0,
        }.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        get = lambda name: self.get_parameter(name).value
        self._source, self._target = get('source_frame'), get('target_frame')
        self._timeout = get('stream_timeout_s')
        self._estimate_center = get('estimate_center')
        self._radius = get('fruit_radius_m')
        self._pear_yaw_world = float(get('pear_yaw_world'))
        self._harvest_pub = (self.create_publisher(PoseStamped, '/harvest/pear_pose', 1)
                             if get('publish_harvest_pose') else None)
        if not math.isfinite(self._radius) or self._radius <= 0:
            raise ValueError('fruit_radius_m must be positive and finite.')
        if not math.isfinite(self._pear_yaw_world):
            raise ValueError('pear_yaw_world must be finite.')
        if not self._source or not self._target or not math.isfinite(self._timeout) or self._timeout <= 0:
            raise ValueError('Invalid frame or timeout.')
        # This lookup only depends on the static world -> fixed-camera chain.
        # Do not bind the TF buffer clock to Gazebo: at startup Gazebo can jump
        # simulated time backwards after robot_state_publisher has already sent
        # /tf_static.  A ROS-clock-bound Buffer clears itself on that jump, while
        # the existing transient-local subscription does not receive the same
        # static sample a second time.  The listener still fills this buffer from
        # /tf and /tf_static; omitting node only gives the buffer a system clock
        # (and omits its optional tf2_frames service).
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self._pending, self._received_at = None, None
        self._state, self._reason, self._result = 'WAITING_POINT', None, None
        self._count = 0
        self._point_pub = self.create_publisher(PointStamped, '/orchard/vision/surface_point_world', 1)
        self._marker_pub = self.create_publisher(Marker, '/orchard/vision/surface_marker_world', 1)
        self._status_pub = self.create_publisher(String, '/orchard/vision/world_status', 1)
        self._center_pub = self.create_publisher(PointStamped, '/orchard/vision/center_point_world', 1)
        self._center_marker_pub = self.create_publisher(Marker, '/orchard/vision/center_marker_world', 1)
        self._center_status_pub = self.create_publisher(String, '/orchard/vision/center_status', 1)
        self._center_result = None
        self._input_subscriptions = [
            self.create_subscription(PointStamped, get('point_topic'), self._receive, 1),
            self.create_subscription(Marker, get('source_marker_topic'), self._source_marker, 1),
        ]
        self._watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._retry_timer = self.create_timer(.1, self._retry, clock=self._watchdog_clock)
        self._status_timer = self.create_timer(1., self._report, clock=self._watchdog_clock)

    def _invalidate(self, state, reason=None):
        self._state, self._reason, self._result = state, reason, None
        self._center_result = None
        marker = Marker()
        marker.header.frame_id = self._target
        marker.ns, marker.id, marker.action = 'pear_surface_world', 0, Marker.DELETE
        self._marker_pub.publish(marker)
        marker.ns = 'pear_center_world'
        self._center_marker_pub.publish(marker)
        self._report_center()

    def _source_marker(self, marker):
        if marker.ns == 'pear_surface' and marker.id == 0 and marker.action == Marker.DELETE:
            self._pending = None
            self._invalidate('SOURCE_INVALID', 'The camera estimator invalidated the point.')

    def _receive(self, point):
        self._pending = None
        if point.header.frame_id != self._source or not all(
                math.isfinite(v) for v in (point.point.x, point.point.y, point.point.z)):
            self._invalidate('INVALID_POINT', 'Invalid source frame or coordinates.')
            return
        self._pending, self._received_at = point, time.monotonic()
        self._retry()

    def _retry(self):
        if self._received_at is not None and time.monotonic() - self._received_at > self._timeout:
            self._pending = None
            if self._state != 'STREAM_TIMEOUT':
                self._invalidate('STREAM_TIMEOUT', 'No recent camera point.')
            return
        if self._pending is None:
            return
        point = self._pending
        try:
            transform = self.buffer.lookup_transform(
                self._target, self._source, Time.from_msg(point.header.stamp))
        except TransformException as error:
            if self._state != 'WAITING_TF':
                self._invalidate('WAITING_TF', str(error))
            return
        # No fallback to latest TF; preserve the measurement timestamp.
        result = do_transform_point(point, transform)
        result.header.stamp = point.header.stamp
        if not all(math.isfinite(v) for v in (result.point.x, result.point.y, result.point.z)):
            self._pending = None
            self._invalidate('INVALID_TF', 'Transformed coordinates are nonfinite.')
            return
        self._point_pub.publish(result)
        marker = Marker()
        marker.header = result.header
        marker.ns, marker.id = 'pear_surface_world', 0
        marker.type, marker.action = Marker.SPHERE, Marker.ADD
        marker.pose.position = result.point
        marker.pose.orientation.w = 1.
        marker.scale.x = marker.scale.y = marker.scale.z = .018
        marker.color.g = marker.color.b = marker.color.a = 1.
        self._marker_pub.publish(marker)
        if self._estimate_center:
            t = transform.transform.translation
            try:
                xyz = estimate_center(
                    (result.point.x, result.point.y, result.point.z),
                    (t.x, t.y, t.z), self._radius)
            except ValueError as error:
                self._pending = None
                self._invalidate('INVALID_CENTER', str(error))
                return
            center = PointStamped()
            center.header = result.header
            center.point.x, center.point.y, center.point.z = xyz
            self._center_pub.publish(center)
            center_marker = Marker()
            center_marker.header = center.header
            center_marker.ns, center_marker.id = 'pear_center_world', 0
            center_marker.type, center_marker.action = Marker.SPHERE, Marker.ADD
            center_marker.pose.position = center.point
            center_marker.pose.orientation.w = 1.0
            center_marker.scale.x = center_marker.scale.y = center_marker.scale.z = 0.025
            center_marker.color.r, center_marker.color.g, center_marker.color.a = 1.0, 0.45, 1.0
            self._center_marker_pub.publish(center_marker)
            self._center_result = xyz
        self._state, self._reason, self._result = 'VALID', None, result
        self._count += 1
        self._pending = None

        self._report_center()
        if self._harvest_pub is not None and self._center_result is not None:
            pose = PoseStamped()
            pose.header = result.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self._center_result
            # Position is observed by RGB-D. The vertical fruit's yaw is
            # supplied by the known scenario, not estimated by this camera.
            pose.pose.orientation.z = math.sin(self._pear_yaw_world / 2.0)
            pose.pose.orientation.w = math.cos(self._pear_yaw_world / 2.0)
            self._harvest_pub.publish(pose)

    def _report(self):
        self._retry()
        result = self._result
        self._status_pub.publish(String(data=json.dumps({
            'state': self._state, 'valid': self._state == 'VALID', 'reason': self._reason,
            'transformed_points': self._count, 'frame': self._target,
            'received_age_s': None if self._received_at is None else round(time.monotonic()-self._received_at, 3),
            'point': None if result is None else [result.point.x, result.point.y, result.point.z],
        })))
        self._report_center()

    def _report_center(self):
        stamp = None if self._result is None else self._result.header.stamp
        self._center_status_pub.publish(String(data=json.dumps({
            'state': self._state if self._estimate_center else 'DISABLED',
            'valid': self._estimate_center and self._state == 'VALID' and self._center_result is not None,
            'reason': self._reason, 'method': 'surface_plus_radius_along_camera_ray',
            'radius_m': self._radius, 'frame': self._target,
            'point': self._center_result,
            'stamp': None if stamp is None else {'sec': stamp.sec, 'nanosec': stamp.nanosec},
            'received_age_s': None if self._received_at is None else round(time.monotonic()-self._received_at, 3),
        })))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PearWorldTransformer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

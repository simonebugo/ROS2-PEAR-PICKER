#!/usr/bin/env python3
"""Check camera transport and TF; the marker is a test ray, not a detection."""

import json
import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


def validate_streams(messages, expected_frame):
    errors = []
    for name in ('rgb', 'depth', 'info'):
        msg = messages.get(name)
        if msg is None:
            errors.append(f'{name}: no message')
            continue
        if msg.header.frame_id != expected_frame:
            errors.append(f'{name}: unexpected frame {msg.header.frame_id!r}')
        if msg.width <= 0 or msg.height <= 0:
            errors.append(f'{name}: invalid dimensions')

    for name, encoding, bytes_per_pixel in (('rgb', 'rgb8', 3), ('depth', '32FC1', 4)):
        msg = messages.get(name)
        if msg is not None:
            if msg.encoding != encoding:
                errors.append(f'{name}: encoding {msg.encoding!r}, expected {encoding}')
            if msg.step < msg.width * bytes_per_pixel or len(msg.data) < msg.height * msg.step:
                errors.append(f'{name}: incomplete image buffer')

    info = messages.get('info')
    if info is not None:
        if (len(info.k) != 9 or not all(math.isfinite(x) for x in info.k)
                or info.k[0] <= 0 or info.k[4] <= 0):
            errors.append('info: invalid K intrinsics')
        for name in ('rgb', 'depth'):
            msg = messages.get(name)
            if msg is not None and (msg.width, msg.height) != (info.width, info.height):
                errors.append(f'{name}: dimensions differ from CameraInfo')
    return errors


class CameraPipelineCheck(Node):
    TOPICS = {
        'rgb': '/orchard/camera/rgb/image_raw',
        'depth': '/orchard/camera/depth/image_raw',
        'info': '/orchard/camera/camera_info',
    }

    def __init__(self):
        super().__init__('camera_pipeline_check')
        self.declare_parameter('camera_frame', 'table_camera_link_optical')
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('stream_timeout_s', 10.0)
        self._messages = {}
        self._received_at = {}
        self._stamps = {name: deque(maxlen=30) for name in ('rgb', 'depth')}
        self._last_errors = None
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._camera_subscriptions = [
            self.create_subscription(
                CameraInfo if name == 'info' else Image, topic,
                lambda msg, stream=name: self._receive(stream, msg), qos_profile_sensor_data,
            )
            for name, topic in self.TOPICS.items()
        ]
        self._status_pub = self.create_publisher(String, '/orchard/camera/status', 10)
        self._marker_pub = self.create_publisher(Marker, '/orchard/camera/optical_axis', 10)
        # Check arrival in wall time so paused/slow simulation is diagnosable.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(2.0, self._check, clock=self._steady_clock)

    def _receive(self, name, msg):
        self._messages[name] = msg
        self._received_at[name] = time.monotonic()
        if name in self._stamps:
            stamp = msg.header.stamp
            value = stamp.sec * 1_000_000_000 + stamp.nanosec
            if self._stamps[name] and value < self._stamps[name][-1]:
                self._stamps[name].clear()
            self._stamps[name].append(value)

    def _check(self):
        camera_frame = self.get_parameter('camera_frame').value
        target_frame = self.get_parameter('target_frame').value
        timeout = float(self.get_parameter('stream_timeout_s').value)
        errors = validate_streams(self._messages, camera_frame)
        now = time.monotonic()
        for name, last in self._received_at.items():
            if now - last > timeout:
                errors.append(f'{name}: stream stopped for over {timeout:g} wall seconds')
        for name, topic in self.TOPICS.items():
            if len(self.get_publishers_info_by_topic(topic)) > 1:
                errors.append(f'{name}: multiple publishers; check active launches')

        paired = set(self._stamps['rgb']) & set(self._stamps['depth'])
        if 'rgb' in self._messages and 'depth' in self._messages and not paired:
            errors.append('RGB/depth: no matching timestamp pair in the last 30 frames')
        try:
            transform = self._buffer.lookup_transform(target_frame, camera_frame, Time())
            self._publish_axis(transform, target_frame, camera_frame)
        except TransformException:
            errors.append(f'TF {target_frame} <- {camera_frame} unavailable')

        self._status_pub.publish(String(data=json.dumps({
            'ready': not errors, 'errors': errors, 'paired_frames': len(paired),
            'camera_frame': camera_frame, 'target_frame': target_frame,
        }, ensure_ascii=False)))
        if tuple(errors) != self._last_errors:
            if errors:
                self.get_logger().warning('Camera not ready: ' + '; '.join(errors))
            else:
                self.get_logger().info('CAMERA READY: RGB, depth, CameraInfo, timestamps and TF are consistent.')
            self._last_errors = tuple(errors)

    def _publish_axis(self, transform, target_frame, camera_frame):
        probe = PointStamped()
        probe.header.frame_id = camera_frame
        probe.point.z = 1.0
        tip = do_transform_point(probe, transform).point
        origin = transform.transform.translation
        marker = Marker()
        marker.header.frame_id = target_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'camera_optical_axis'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.points = [Point(x=origin.x, y=origin.y, z=origin.z), tip]
        marker.scale.x, marker.scale.y, marker.scale.z = 0.015, 0.035, 0.06
        marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 1.0
        self._marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = CameraPipelineCheck()
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

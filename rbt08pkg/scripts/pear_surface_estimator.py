#!/usr/bin/env python3
"""Exact timestamp mask/depth pairing and observed camera-frame surface points."""

from collections import OrderedDict
from dataclasses import asdict
import json
import math
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from pear_depth import DepthConfig, Intrinsics, estimate_surface


class PearSurfaceEstimator(Node):
    def __init__(self):
        super().__init__('pear_surface_estimator')
        readonly = ParameterDescriptor(read_only=True)
        defaults = {
            'mask_topic': '/orchard/vision/target_mask',
            'depth_topic': '/orchard/camera/depth/image_raw',
            'camera_info_topic': '/orchard/camera/camera_info',
            'camera_frame': 'table_camera_link_optical',
            'queue_size': 20, 'stream_timeout_s': 10.0,
            **asdict(DepthConfig()),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, readonly)
        value = lambda name: self.get_parameter(name).value
        self._config = DepthConfig(**{name: value(name) for name in asdict(DepthConfig())})
        self._config.validate()
        self._timeout, self._queue_size = value('stream_timeout_s'), value('queue_size')
        if not math.isfinite(self._timeout) or self._timeout <= 0 or self._queue_size < 1:
            raise ValueError('Timeout e queue_size devono essere positivi.')
        self._frame = value('camera_frame')
        self._queues = {'mask': OrderedDict(), 'depth': OrderedDict()}
        self._stamps = {}
        self._info = None
        self._bridge = CvBridge()
        cv2.setNumThreads(1)
        self._state, self._reason = 'WAITING_PAIR', None
        self._result, self._header = None, None
        self._last_pair, self._last_valid = None, None
        self._pairs = 0
        self._processed_stamp = None
        self._point_pub = self.create_publisher(PointStamped, '/orchard/vision/surface_point', 1)
        self._marker_pub = self.create_publisher(Marker, '/orchard/vision/surface_marker', 1)
        self._status_pub = self.create_publisher(String, '/orchard/vision/depth_status', 1)
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._input_subscriptions = [
            self.create_subscription(Image, value('mask_topic'), lambda msg: self._receive('mask', msg), qos),
            self.create_subscription(Image, value('depth_topic'), lambda msg: self._receive('depth', msg), qos),
            self.create_subscription(CameraInfo, value('camera_info_topic'), self._receive_info, qos),
        ]
        self._watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(1.0, self._report, clock=self._watchdog_clock)
        self.get_logger().info('Stima superficie 3D pronta: sincronizzazione esatta mask/depth, frame camera.')

    def _receive_info(self, message):
        self._info = message
        self._match()

    def _invalidate(self, state, reason=None):
        self._state, self._reason, self._result = state, reason, None
        marker = Marker()
        if self._header is not None:
            marker.header = self._header
        marker.ns, marker.id, marker.action = 'pear_surface', 0, Marker.DELETE
        self._marker_pub.publish(marker)

    def _receive(self, stream, message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if stamp < self._stamps.get(stream, stamp):
            for queue in self._queues.values():
                queue.clear()
            self._stamps.clear()
            self._last_pair = self._last_valid = None
            self._processed_stamp = None
            self._invalidate('TIME_RESET')
        self._stamps[stream] = stamp
        if message.header.frame_id != self._frame:
            self._invalidate('INVALID_FRAME', f'{stream}: {message.header.frame_id}')
            return
        if self._processed_stamp is not None and stamp <= self._processed_stamp:
            return
        queue = self._queues[stream]
        queue[stamp] = (message, time.monotonic())
        while len(queue) > self._queue_size:
            queue.popitem(last=False)
        self._match()

    def _match(self):
        if self._info is None:
            return
        shared = set(self._queues['mask']) & set(self._queues['depth'])
        if not shared:
            return
        stamp = max(shared)
        self._processed_stamp = stamp
        mask_msg, mask_at = self._queues['mask'][stamp]
        depth_msg, depth_at = self._queues['depth'][stamp]
        # Drop older pairs: never publish an older point after a newer frame.
        for queue in self._queues.values():
            for key in list(queue):
                if key <= stamp:
                    del queue[key]
        self._header = mask_msg.header
        self._pairs += 1
        self._last_pair = time.monotonic()
        if self._last_pair - min(mask_at, depth_at) > self._timeout:
            self._invalidate('STALE_PAIR')
            return
        try:
            info = self._info
            if info.header.frame_id != self._frame:
                raise ValueError('CameraInfo nel frame errato.')
            if mask_msg.encoding != 'mono8' or depth_msg.encoding != '32FC1':
                raise ValueError('Attesi mask mono8 e depth 32FC1 (metri).')
            # Current Gazebo stream is undistorted. Reject incompatible raw
            # calibration rather than silently using pinhole formulas on it.
            if (info.distortion_model != 'plumb_bob' or
                    any(not math.isfinite(v) or abs(v) > 1e-10 for v in info.d)):
                raise ValueError('Richieste immagini senza distorsione; rettificare una futura camera reale.')
            if not all(math.isfinite(v) for v in info.k) or abs(info.k[8] - 1) > 1e-8:
                raise ValueError('Matrice K non valida.')
            if any(abs(info.k[i]) > 1e-8 for i in (1, 3, 6, 7)):
                raise ValueError('K non compatibile con il modello pinhole senza skew.')
            camera = Intrinsics(info.width, info.height, info.k[0], info.k[4], info.k[2], info.k[5])
            mask = self._bridge.imgmsg_to_cv2(mask_msg, desired_encoding='passthrough')
            depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            self._result = estimate_surface(mask, depth, camera, self._config)
            point = PointStamped()
            point.header = mask_msg.header
            point.point.x, point.point.y, point.point.z = self._result['point']
            self._point_pub.publish(point)
            marker = Marker()
            marker.header, marker.ns, marker.id = mask_msg.header, 'pear_surface', 0
            marker.type, marker.action = Marker.SPHERE, Marker.ADD
            marker.pose.position = point.point
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.025
            marker.color.r, marker.color.b, marker.color.a = 1.0, 1.0, 1.0
            self._marker_pub.publish(marker)
            self._state, self._reason, self._last_valid = 'VALID', None, time.monotonic()
        except Exception as error:
            self._invalidate('NO_TARGET' if str(error) == 'NO_TARGET' else 'INVALID_DEPTH', str(error))

    def _report(self):
        now = time.monotonic()
        age = None if self._last_valid is None else now - self._last_valid
        pair_age = None if self._last_pair is None else now - self._last_pair
        for queue in self._queues.values():
            for key, (_, received_at) in list(queue.items()):
                if now - received_at > self._timeout:
                    del queue[key]
        if pair_age is not None and pair_age > self._timeout:
            self._invalidate('STREAM_TIMEOUT', 'Nessuna coppia esatta recente.')
        elif self._state == 'VALID' and age is not None and age > self._timeout:
            self._invalidate('STREAM_TIMEOUT', 'Nessuna stima valida recente.')
        self._status_pub.publish(String(data=json.dumps({
            'state': self._state, 'valid': self._state == 'VALID', 'reason': self._reason,
            'paired_frames': self._pairs, 'valid_age_s': None if age is None else round(age, 3),
            'camera_info_received': self._info is not None,
            'pending_masks': len(self._queues['mask']), 'pending_depth': len(self._queues['depth']),
            'frame': self._frame, 'estimate': self._result,
        })))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PearSurfaceEstimator()
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

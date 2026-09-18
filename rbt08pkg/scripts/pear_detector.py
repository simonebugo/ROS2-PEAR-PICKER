#!/usr/bin/env python3
"""ROS RGB -> HSV masks and one 2D target; no neural network required."""

from dataclasses import asdict
import json
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from pear_segmentation import SegmentationConfig, annotate, segment_pear


class PearDetector(Node):
    def __init__(self):
        super().__init__('pear_detector')
        readonly = ParameterDescriptor(read_only=True)
        topics = {
            'rgb_topic': '/orchard/camera/rgb/image_raw',
            'detections_topic': '/orchard/vision/detections',
            'annotated_topic': '/orchard/vision/image_annotated',
            'mask_topic': '/orchard/vision/mask',
            'target_mask_topic': '/orchard/vision/target_mask',
            'status_topic': '/orchard/vision/status',
        }
        for name, value in topics.items():
            self.declare_parameter(name, value, readonly)
        for name, value in asdict(SegmentationConfig()).items():
            self.declare_parameter(name, list(value) if isinstance(value, tuple) else value)
        self.declare_parameter('stream_timeout_s', 10.0)
        self.declare_parameter('max_rate_hz', 5.0, readonly)
        self._config().validate()
        self._validate_timeout(self.get_parameter('stream_timeout_s').value)
        rate = self.get_parameter('max_rate_hz').value
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('max_rate_hz deve essere finito e positivo.')
        self.add_on_set_parameters_callback(self._validate_parameters)
        cv2.setNumThreads(1)
        self._bridge = CvBridge()
        self._latest = None
        self._last_received = None
        self._last_message = None
        self._state = 'WAITING_IMAGE'
        self._error = None
        self._frames = 0
        self._processing_ms = None
        self._target = None
        self._valid_candidates = 0
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        topic = lambda name: self.get_parameter(name).value
        self._detections = self.create_publisher(Detection2DArray, topic('detections_topic'), 1)
        self._annotated = self.create_publisher(Image, topic('annotated_topic'), image_qos)
        self._mask = self.create_publisher(Image, topic('mask_topic'), image_qos)
        self._target_mask = self.create_publisher(Image, topic('target_mask_topic'), image_qos)
        self._status = self.create_publisher(String, topic('status_topic'), 1)
        self._subscription = self.create_subscription(
            Image, topic('rgb_topic'), self._receive, image_qos)
        self._wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(1.0 / rate, self._process, clock=self._wall_clock)
        self._watchdog = self.create_timer(1.0, self._report_status, clock=self._wall_clock)
        self.get_logger().info(
            f'Segmentazione HSV pronta: attendo {topic("rgb_topic")}; '
            'soglie modificabili con parametri ROS, nessun peso o venv richiesto.')

    def _config(self, overrides=None):
        values = {name: self.get_parameter(name).value for name in asdict(SegmentationConfig())}
        values.update(overrides or {})
        return SegmentationConfig(**values)

    @staticmethod
    def _validate_timeout(value):
        if not math.isfinite(value) or value <= 0:
            raise ValueError('stream_timeout_s deve essere finito e positivo.')

    def _validate_parameters(self, parameters):
        overrides = {p.name: p.value for p in parameters if p.name in asdict(SegmentationConfig())}
        try:
            self._config(overrides).validate()
            for parameter in parameters:
                if parameter.name == 'stream_timeout_s':
                    self._validate_timeout(parameter.value)
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def _receive(self, message):
        self._latest = message
        self._last_message = message
        self._last_received = time.monotonic()

    def _publish_image(self, publisher, image, encoding, header):
        message = self._bridge.cv2_to_imgmsg(image, encoding=encoding)
        message.header = header
        publisher.publish(message)

    def _clear_target(self, header):
        self._target = None
        self._valid_candidates = 0
        empty = Detection2DArray()
        empty.header = header
        self._detections.publish(empty)
        # Keep the last source stamp: a stopped camera does not create new frames.
        if self._last_message is not None:
            h, w = self._last_message.height, self._last_message.width
            if 0 < h <= 4096 and 0 < w <= 4096:
                blank = np.zeros((h, w), dtype=np.uint8)
                self._publish_image(self._mask, blank, 'mono8', header)
                self._publish_image(self._target_mask, blank, 'mono8', header)
                image = np.zeros((h, w, 3), dtype=np.uint8)
                cv2.putText(image, self._state, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (255, 255, 255), 2)
                self._publish_image(self._annotated, image, 'bgr8', header)

    def _process(self):
        message = self._latest
        if message is None:
            return
        self._latest = None
        start = time.monotonic()
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            result = segment_pear(image, self._config())
            detections = Detection2DArray()
            detections.header = message.header
            candidate = result.candidate
            if candidate is not None:
                detection = Detection2D()
                detection.header = message.header
                detection.bbox.center.position.x, detection.bbox.center.position.y = candidate.bbox_center
                detection.bbox.size_x, detection.bbox.size_y = float(candidate.width), float(candidate.height)
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = 'pear'
                # Geometric occupancy score, NOT a learned class probability.
                hypothesis.hypothesis.score = candidate.fill_ratio
                detection.results.append(hypothesis)
                detections.detections.append(detection)
            self._publish_image(self._annotated, annotate(image, result), 'bgr8', message.header)
            self._publish_image(self._mask, result.mask, 'mono8', message.header)
            self._publish_image(self._target_mask, result.target_mask, 'mono8', message.header)
            self._detections.publish(detections)
            self._target = asdict(candidate) if candidate else None
            self._valid_candidates = result.valid_candidates
            self._state = 'DETECTED' if candidate else 'NO_TARGET'
            self._error = None
        except Exception as error:
            if str(error) != self._error:
                self.get_logger().error(f'Segmentazione fallita: {error}')
            self._error = str(error)
            self._state = 'ERROR'
            self._clear_target(message.header)
        self._frames += 1
        self._processing_ms = round((time.monotonic() - start) * 1000, 2)

    def _report_status(self):
        age = None if self._last_received is None else time.monotonic() - self._last_received
        timeout = self.get_parameter('stream_timeout_s').value
        if age is not None and age > timeout and self._state != 'STREAM_TIMEOUT':
            self._state = 'STREAM_TIMEOUT'
            self._latest = None
            self._clear_target(self._last_message.header)
            self.get_logger().warning('RGB fermo: target invalidato, attendo nuove immagini.')
        self._status.publish(String(data=json.dumps({
            'method': 'hsv', 'state': self._state,
            'ready': self._state in ('DETECTED', 'NO_TARGET'),
            'detected': self._state == 'DETECTED',
            'rgb_age_s': None if age is None else round(age, 3),
            'processed_frames': self._frames, 'processing_ms': self._processing_ms,
            'valid_candidates': self._valid_candidates, 'target': self._target,
            'error': self._error,
        })))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PearDetector()
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

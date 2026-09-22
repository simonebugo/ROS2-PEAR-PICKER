"""Save one RGB frame, masks and geometry for reproducible HSV tuning."""

import argparse
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
import time

import cv2
import yaml

from segmentation_paths import add_segmentation_path

add_segmentation_path()
from pear_segmentation import SegmentationConfig, annotate, segment_pear


def capture_rgb(topic, timeout):
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    rclpy.init(args=[])
    node = Node('hsv_capture')
    received = []
    subscription = node.create_subscription(Image, topic, received.append, qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + timeout
        print(f'Waiting for an RGB frame on {topic} (timeout {timeout:g}s)...')
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not received:
            raise RuntimeError('No RGB received; check camera, topic and bridge.')
        message = received[0]
        image = CvBridge().imgmsg_to_cv2(message, desired_encoding='bgr8')
        header = {'frame_id': message.header.frame_id,
                  'sec': message.header.stamp.sec, 'nanosec': message.header.stamp.nanosec}
        return image, header
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', type=Path, help='Saved image; omit to capture an RGB frame from ROS.')
    parser.add_argument('--topic', default='/orchard/camera/rgb/image_raw')
    parser.add_argument('--timeout', type=float, default=60.0)
    parser.add_argument('--config', type=Path, help='pear_segmentation.yaml configuration; use defaults if omitted.')
    parser.add_argument('--hsv-lower', type=int, nargs=3)
    parser.add_argument('--hsv-upper', type=int, nargs=3)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent / 'runs' / 'hsv_debug')
    args = parser.parse_args()
    settings = {}
    if args.config:
        params = yaml.safe_load(args.config.read_text())['pear_detector']['ros__parameters']
        settings = {key: value for key, value in params.items() if key in asdict(SegmentationConfig())}
    for key in ('hsv_lower', 'hsv_upper'):
        if getattr(args, key) is not None:
            settings[key] = getattr(args, key)
    config = SegmentationConfig(**settings)
    config.validate()
    if args.image:
        image = cv2.imread(str(args.image))
        if image is None:
            parser.error(f'Cannot read image: {args.image}')
        header = None
    else:
        image, header = capture_rgb(args.topic, args.timeout)
    start = time.perf_counter()
    result = segment_pear(image, config)
    elapsed = (time.perf_counter() - start) * 1000
    directory = args.output / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    directory.mkdir(parents=True, exist_ok=False)
    for name, output in (('rgb', image), ('mask', result.mask),
                         ('target_mask', result.target_mask), ('annotated', annotate(image, result))):
        if not cv2.imwrite(str(directory / f'{name}.png'), output):
            raise RuntimeError(f'Cannot save {name}.png')
    report = {
        'header': header, 'config': asdict(config), 'processing_ms': round(elapsed, 2),
        'valid_candidates': result.valid_candidates,
        'target': asdict(result.candidate) if result.candidate else None,
    }
    (directory / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    print(f'Frame and masks saved to: {directory}')


if __name__ == '__main__':
    main()

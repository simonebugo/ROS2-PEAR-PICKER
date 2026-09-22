"""Exercise the real ROS node on private synthetic topics, without the robot."""
import json
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from segmentation_paths import add_segmentation_path

add_segmentation_path()
from pear_surface_estimator import PearSurfaceEstimator


def main():
    prefix = f'/depth_selftest_{os.getpid()}'
    rclpy.init(args=['--ros-args', '-p', f'mask_topic:={prefix}/mask',
                    '-p', f'depth_topic:={prefix}/depth', '-p', f'camera_info_topic:={prefix}/info',
                    '-p', 'stream_timeout_s:=2.0', '-p', 'use_sim_time:=false',
                    '-r', f'/orchard/vision/surface_point:={prefix}/point',
                    '-r', f'/orchard/vision/surface_marker:={prefix}/marker',
                    '-r', f'/orchard/vision/depth_status:={prefix}/status'])
    estimator = PearSurfaceEstimator()
    fixture = Node('depth_fixture')
    executor = SingleThreadedExecutor()
    executor.add_node(estimator)
    executor.add_node(fixture)
    bridge = CvBridge()
    points, markers, statuses = [], [], []
    subscriptions = [
        fixture.create_subscription(PointStamped, '/orchard/vision/surface_point', points.append, 10),
        fixture.create_subscription(Marker, '/orchard/vision/surface_marker', markers.append, 10),
        fixture.create_subscription(String, '/orchard/vision/depth_status',
                                    lambda msg: statuses.append(json.loads(msg.data)), 10),
    ]
    pubs = {name: fixture.create_publisher(kind, f'{prefix}/{name}', 10)
            for name, kind in [('mask', Image), ('depth', Image), ('info', CameraInfo)]}

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.02)

    def wait_for(predicate, label, timeout=5):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            executor.spin_once(timeout_sec=.02)
        if not predicate():
            raise AssertionError(label)

    mask = np.zeros((80, 100), np.uint8)
    cv2.circle(mask, (65, 30), 15, 255, -1)
    depth = np.full(mask.shape, 1.2, np.float32)
    info = CameraInfo(width=100, height=80, distortion_model='plumb_bob')
    info.header.frame_id = 'table_camera_link_optical'
    info.k = [100., 0., 50., 0., 200., 40., 0., 0., 1.]
    info.d = [0.] * 5

    def image(array, encoding, stamp):
        msg = bridge.cv2_to_imgmsg(array, encoding=encoding)
        msg.header.frame_id = info.header.frame_id
        msg.header.stamp.sec = stamp
        return msg

    def pair(stamp, mask_array=mask, depth_array=depth):
        pubs['mask'].publish(image(mask_array, 'mono8', stamp))
        pubs['depth'].publish(image(depth_array, '32FC1', stamp))

    try:
        wait_for(lambda: all(pub.get_subscription_count() for pub in pubs.values()), 'Discovery failed')
        spin(.3)
        pubs['info'].publish(info)
        pubs['mask'].publish(image(mask, 'mono8', 100))
        pubs['depth'].publish(image(depth, '32FC1', 99))
        spin(.3)
        assert not points, 'Timestamp diversi abbinati erroneamente'
        pubs['depth'].publish(image(depth, '32FC1', 100))
        wait_for(lambda: bool(points), 'No point published for an exact pair')
        p = points[-1]
        assert p.header.frame_id == info.header.frame_id and p.header.stamp.sec == 100
        np.testing.assert_allclose([p.point.x, p.point.y, p.point.z], [.18, -.06, 1.2], atol=1e-6)
        count = len(points)
        pair(102, mask_array=np.zeros_like(mask))
        wait_for(lambda: estimator._state == 'NO_TARGET', 'Empty mask did not invalidate the target')
        spin(.1)
        assert len(points) == count and markers[-1].action == Marker.DELETE
        pair(103, depth_array=np.full_like(depth, np.nan))
        wait_for(lambda: estimator._state == 'INVALID_DEPTH', 'NaN values were not rejected')
        pair(104)
        wait_for(lambda: len(points) > count, 'Recovery failed')
        wait_for(lambda: statuses and statuses[-1]['state'] == 'STREAM_TIMEOUT', 'Watchdog failed', 5)
        assert not statuses[-1]['valid']
        count = len(points)
        pair(105)
        wait_for(lambda: len(points) > count, 'Recovery after timeout failed')
        wrong = image(mask, 'mono8', 106)
        wrong.header.frame_id = 'wrong_frame'
        pubs['mask'].publish(wrong)
        wait_for(lambda: estimator._state == 'INVALID_FRAME', 'Incorrect frame was not rejected')
        bad = image(depth, '32FC1', 107)
        bad.encoding = '16UC1'
        pubs['mask'].publish(image(mask, 'mono8', 107))
        pubs['depth'].publish(bad)
        wait_for(lambda: estimator._state == 'INVALID_DEPTH', 'Incorrect encoding was not rejected')
        count = len(points)
        pair(50)
        wait_for(lambda: len(points) > count, 'Recovery after time reset failed')
        assert points[-1].header.stamp.sec == 50
        print('DEPTH PREFLIGHT OK: timestamp, deprojection, empty mask, NaN, frame, encoding, watchdog, reset, recovery.')
    finally:
        executor.shutdown()
        estimator.destroy_node()
        fixture.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

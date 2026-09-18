"""ROS/OpenCV preflight and isolated topic test, without Gazebo or robot commands."""

import json
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from segmentation_paths import add_segmentation_path

MODULE_PATH = add_segmentation_path()
from pear_detector import PearDetector


def require(condition, explanation):
    if not condition:
        raise RuntimeError(explanation)


def main():
    print(f'OpenCV {cv2.__version__}; NumPy {np.__version__}; ROS import OK')
    print(f'Moduli verificati: {MODULE_PATH}')
    namespace = f'/hsv_selftest_{os.getpid()}'
    args = ['--ros-args', '-r', f'__ns:={namespace}',
            '-p', 'use_sim_time:=false', '-p', 'stream_timeout_s:=0.6']
    for parameter, suffix in (
        ('rgb_topic', 'rgb'), ('detections_topic', 'detections'),
        ('annotated_topic', 'annotated'), ('mask_topic', 'mask'),
        ('target_mask_topic', 'target_mask'), ('status_topic', 'status'),
    ):
        args += ['-p', f'{parameter}:={namespace}/{suffix}']
    rclpy.init(args=args)
    detector = fixture = executor = None
    try:
        detector = PearDetector()
        fixture = Node('fixture')
        executor = SingleThreadedExecutor()
        executor.add_node(detector)
        executor.add_node(fixture)
        bridge = CvBridge()
        received = {}
        subscription_handles = []
        for suffix, message_type, qos in (
            ('detections', Detection2DArray, 1), ('annotated', Image, qos_profile_sensor_data),
            ('mask', Image, qos_profile_sensor_data), ('target_mask', Image, qos_profile_sensor_data),
            ('status', String, 1),
        ):
            subscription_handles.append(fixture.create_subscription(
                message_type, f'{namespace}/{suffix}',
                lambda message, key=suffix: received.__setitem__(key, message), qos))
        publisher = fixture.create_publisher(Image, f'{namespace}/rgb', qos_profile_sensor_data)

        def wait_for(predicate, description, publish=None, timeout=10.0):
            deadline = time.monotonic() + timeout
            next_publish = 0.0
            while time.monotonic() < deadline:
                now = time.monotonic()
                if publish is not None and now >= next_publish:
                    publisher.publish(publish)
                    next_publish = now + 0.15
                executor.spin_once(timeout_sec=0.05)
                if predicate():
                    return
            raise RuntimeError(f'Timeout test: {description}')

        wait_for(lambda: publisher.get_subscription_count() > 0, 'discovery ROS')
        positive = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.ellipse(positive, (160, 120), (20, 30), 0, 0, 360, (13, 209, 255), -1)
        # Explicit rgb8 source checks the RGB->BGR conversion used by the node.
        source = bridge.cv2_to_imgmsg(cv2.cvtColor(positive, cv2.COLOR_BGR2RGB), 'rgb8')
        source.header.frame_id = 'synthetic_camera_optical'
        source.header.stamp = fixture.get_clock().now().to_msg()

        def matching(key, message):
            value = received.get(key)
            return (value is not None and value.header.stamp == message.header.stamp
                    and value.header.frame_id == message.header.frame_id)

        def all_matching(message):
            return all(matching(key, message) for key in ('detections', 'annotated', 'mask', 'target_mask'))

        wait_for(lambda: all_matching(source), 'frame positivo e header', source)
        detection_array = received['detections']
        require(len(detection_array.detections) == 1, 'Attesa una detection per il frutto giallo.')
        detection = detection_array.detections[0]
        require(detection.header == source.header, 'Header della singola detection alterato.')
        require(abs(detection.bbox.center.position.x - 160) < 2, 'Centro bbox errato.')
        require(detection.results[0].hypothesis.class_id == 'pear', 'Classe inattesa.')
        require(received['annotated'].encoding == 'bgr8', 'Encoding overlay errato.')
        require(received['target_mask'].encoding == 'mono8', 'Encoding maschera errato.')
        require(np.count_nonzero(bridge.imgmsg_to_cv2(received['target_mask'])) > 0, 'Maschera vuota.')
        print('OK: RGB->BGR, detection, maschere, overlay, timestamp e frame originali.')

        rejected = detector.set_parameters([Parameter('opening_size', value=2)])[0]
        require(not rejected.successful, 'Kernel non valido accettato.')
        require(detector.get_parameter('opening_size').value == 3, 'Parametro modificato dopo rifiuto.')
        print('OK: parametri non validi rifiutati.')

        negative = bridge.cv2_to_imgmsg(np.zeros_like(positive), 'bgr8')
        negative.header.frame_id = source.header.frame_id
        negative.header.stamp = fixture.get_clock().now().to_msg()
        wait_for(lambda: all_matching(negative), 'frame senza frutto', negative)
        require(not received['detections'].detections, 'Target precedente rimasto dopo frame negativo.')
        require(not np.any(bridge.imgmsg_to_cv2(received['target_mask'])), 'Maschera precedente rimasta.')
        print('OK: nessuna detection e maschera nera quando il frutto scompare.')

        source.header.stamp = fixture.get_clock().now().to_msg()
        wait_for(lambda: all_matching(source), 'nuovo target', source)
        require(received['detections'].detections, 'Target non recuperato.')

        def stream_expired():
            status = json.loads(received['status'].data) if 'status' in received else {}
            return (status.get('state') == 'STREAM_TIMEOUT'
                    and not received['detections'].detections
                    and not np.any(bridge.imgmsg_to_cv2(received['target_mask'])))

        wait_for(stream_expired, 'timeout con invalidazione target', timeout=5.0)
        print('OK: interruzione RGB invalida detection e maschere.')

        malformed = Image()
        malformed.header.frame_id = source.header.frame_id
        malformed.header.stamp = fixture.get_clock().now().to_msg()
        malformed.height, malformed.width = 240, 320
        malformed.encoding = 'invalid_for_test'

        def conversion_failed():
            status = json.loads(received['status'].data) if 'status' in received else {}
            return (matching('detections', malformed) and not received['detections'].detections
                    and status.get('state') == 'ERROR')

        wait_for(conversion_failed, 'gestione errore conversione (errore log atteso)', malformed)
        source.header.stamp = fixture.get_clock().now().to_msg()
        wait_for(lambda: all_matching(source), 'ripresa dopo errore', source)
        require(received['detections'].detections, 'Ripresa dopo errore fallita.')
        print('PREFLIGHT OK: detector HSV verificato su topic ROS isolati, senza Gazebo.')
    finally:
        if executor is not None:
            executor.shutdown()
        if fixture is not None:
            fixture.destroy_node()
        if detector is not None:
            detector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

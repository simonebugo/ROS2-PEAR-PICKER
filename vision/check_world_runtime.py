"""ROS TF2 preflight on private topics: no Gazebo and no robot commands."""
import os
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.clock import ClockType
from rosgraph_msgs.msg import Clock
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker
from segmentation_paths import add_segmentation_path

add_segmentation_path()
from pear_world_transformer import PearWorldTransformer


def main():
    root = f'/world_selftest_{os.getpid()}'
    source, target = f'camera_test_{os.getpid()}', f'world_test_{os.getpid()}'
    clock_topic = f'{root}/clock'
    rclpy.init(args=['--ros-args', '-p', f'source_frame:={source}', '-p', f'target_frame:={target}',
                    '-p', f'point_topic:={root}/camera_point', '-p', f'source_marker_topic:={root}/camera_marker',
                    '-p', 'stream_timeout_s:=3.0', '-p', 'use_sim_time:=true',
                    '-r', f'/orchard/vision/surface_point_world:={root}/point',
                    '-r', f'/orchard/vision/surface_marker_world:={root}/marker',
                    '-r', f'/orchard/vision/world_status:={root}/status',
                    '-r', f'/orchard/vision/center_point_world:={root}/center',
                    '-r', f'/orchard/vision/center_marker_world:={root}/center_marker',
                    '-r', f'/orchard/vision/center_status:={root}/center_status',
                    '-r', f'/tf:={root}/tf', '-r', f'/tf_static:={root}/tf_static',
                    '-r', f'/clock:={clock_topic}'])
    node = PearWorldTransformer()
    fixture = Node('world_fixture')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(fixture)
    points, markers = [], []
    centers, center_markers = [], []
    subscriptions = [fixture.create_subscription(PointStamped, f'{root}/point', points.append, 10),
                     fixture.create_subscription(Marker, f'{root}/marker', markers.append, 10)]
    point_pub = fixture.create_publisher(PointStamped, f'{root}/camera_point', 10)
    subscriptions.extend([
        fixture.create_subscription(PointStamped, f'{root}/center', centers.append, 10),
        fixture.create_subscription(Marker, f'{root}/center_marker', center_markers.append, 10),
    ])
    marker_pub = fixture.create_publisher(Marker, f'{root}/camera_marker', 10)
    clock_pub = fixture.create_publisher(Clock, clock_topic, 10)
    broadcaster = StaticTransformBroadcaster(fixture)

    def wait(predicate, label, seconds=5):
        end = time.monotonic() + seconds
        while not predicate() and time.monotonic() < end:
            executor.spin_once(timeout_sec=.02)
        if not predicate():
            frames = node.buffer.all_frames_as_yaml()
            raise AssertionError(
                f'{label}; state={node._state}; reason={node._reason}; '
                f'tf_static_subscribers={broadcaster.pub_tf.get_subscription_count()}; '
                f'frames={frames!r}')

    def point(stamp):
        p = PointStamped()
        p.header.frame_id, p.header.stamp.sec = source, stamp
        p.header.stamp.nanosec = 123
        p.point.x, p.point.y, p.point.z = 1., 2., 3.
        return p

    try:
        registered = list(node.subscriptions)
        assert node.listener.tf_sub in registered, 'Subscription /tf rimossa dal nodo'
        assert node.listener.tf_static_sub in registered, 'Subscription /tf_static rimossa dal nodo'
        assert any(s.topic_name == clock_topic for s in registered), 'Subscription /clock rimossa dal nodo'
        assert node.get_clock().clock_type == ClockType.ROS_TIME, 'Clock ROS sovrascritto'
        wait(lambda: point_pub.get_subscription_count()
             and marker_pub.get_subscription_count()
             and broadcaster.pub_tf.get_subscription_count()
             and node._point_pub.get_subscription_count()
             and node._marker_pub.get_subscription_count()
             and node._center_pub.get_subscription_count()
             and node._center_marker_pub.get_subscription_count()
             and clock_pub.get_subscription_count() >= 2,
             'Discovery DDS fallita')
        point_pub.publish(point(100))
        wait(lambda: node._state == 'WAITING_TF', 'TF mancante non rilevato')
        assert not points
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = target, source
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = 10., 20., 30.
        tf.transform.rotation.z = tf.transform.rotation.w = math.sqrt(.5)

        broadcaster.sendTransform(tf)
        wait(lambda: bool(points), 'Nessuna trasformazione dopo arrivo TF')
        p = points[-1]
        assert p.header.frame_id == target and p.header.stamp.sec == 100 and p.header.stamp.nanosec == 123
        assert all(abs(a-b) < 1e-9 for a, b in zip((p.point.x, p.point.y, p.point.z), (8., 21., 33.)))
        wait(lambda: bool(centers), 'Centro non pubblicato')
        center = centers[-1]
        expected = (8. - .15/math.sqrt(14), 21. + .075/math.sqrt(14), 33. + .225/math.sqrt(14))
        assert center.header == p.header, 'Timestamp o frame centro non conservato'
        assert all(abs(a-b) < 1e-9 for a, b in zip(
            (center.point.x, center.point.y, center.point.z), expected)), 'Centro geometrico errato'

        # Regression: Gazebo can reset /clock after /tf_static was delivered.
        # The fixed-camera transform must remain usable without restarting this node.
        clock = Clock()
        clock.clock.sec = 20
        clock_pub.publish(clock)
        wait(lambda: node.get_clock().now().nanoseconds == 20_000_000_000,
             'Clock simulato non attivato')
        clock.clock.sec = 1
        clock_pub.publish(clock)
        wait(lambda: node.get_clock().now().nanoseconds == 1_000_000_000,
             'Reset del clock simulato non ricevuto')
        count = len(points)
        point_pub.publish(point(101))
        wait(lambda: len(points) > count, 'TF statica persa dopo reset di /clock')

        wait(lambda: node._state == 'STREAM_TIMEOUT', 'Watchdog fallito')
        wait(lambda: markers and markers[-1].action == Marker.DELETE, 'Marker scaduto non eliminato')
        wait(lambda: center_markers and center_markers[-1].action == Marker.DELETE,
             'Marker centro scaduto non eliminato')
        count = len(points)
        point_pub.publish(point(102))
        wait(lambda: len(points) > count, 'Recupero fallito')
        deleted = Marker()
        deleted.ns, deleted.id, deleted.action = 'pear_surface', 0, Marker.DELETE
        marker_pub.publish(deleted)
        wait(lambda: node._state == 'SOURCE_INVALID', 'Invalidazione sorgente non propagata')
        assert node._center_result is None, 'Centro residuo dopo invalidazione'
        bad = point(103)
        bad.header.frame_id = 'wrong'
        point_pub.publish(bad)
        wait(lambda: node._state == 'INVALID_POINT', 'Frame errato non rifiutato')
        print('WORLD PREFLIGHT OK: TF, centro stimato, timestamp, reset clock, timeout, recupero, invalidazione.')
    finally:
        executor.shutdown()
        node.destroy_node()
        fixture.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

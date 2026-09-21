"""Private ROS preflight: real publishers and trigger checks, no robot motion."""
import os
import time

import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Empty, String
from segmentation_paths import add_segmentation_path

add_segmentation_path()
from pear_world_transformer import PearWorldTransformer
from pick_place_manager import PickPlaceManager


class RecordingManager(PickPlaceManager):
    def __init__(self):
        super().__init__()
        self.started = []

    def start_pick_and_place_sequence(self, *poses):
        self.started.append(poses)


def main():
    prefix = f'/pick_target_test_{os.getpid()}'
    args = ['--ros-args', '-p', 'publish_harvest_pose:=true',
            '-p', 'require_vision_status:=true', '-p', 'use_sim_time:=false',
            '-p', f'point_topic:={prefix}/input', '-p', f'source_marker_topic:={prefix}/input_marker']
    topics = [
        '/harvest/pear_pose', '/harvest/state', '/harvest/trigger',
        '/harvest/pre_grasp_pose', '/harvest/grasp_pose', '/harvest/lift_pose',
        '/harvest/pre_place_pose', '/harvest/place_pose', '/harvest/retreat_pose',
        '/harvest/target_markers', '/harvest/gripper/close_complete', '/planning_scene',
        '/move_action', '/execute_trajectory', '/gripper_controller/follow_joint_trajectory',
        '/compute_cartesian_path', '/compute_ik', '/tf', '/tf_static',
        '/orchard/vision/surface_point_world', '/orchard/vision/surface_marker_world',
        '/orchard/vision/world_status', '/orchard/vision/center_point_world',
        '/orchard/vision/center_marker_world', '/orchard/vision/center_status',
    ]
    for topic in topics:
        args.extend(['-r', f'{topic}:={prefix}{topic}'])
    rclpy.init(args=args)
    world, manager = PearWorldTransformer(), RecordingManager()
    executor = SingleThreadedExecutor()
    executor.add_node(world)
    executor.add_node(manager)

    def wait(predicate, label):
        deadline = time.monotonic() + 5
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        assert predicate(), label

    try:
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'world', 'table_camera_link_optical'
        tf.transform.rotation.w = 1.
        world.buffer.set_transform_static(tf, 'selftest')
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'base_link', 'world'
        tf.transform.rotation.w = 1.
        manager.tf_buffer.set_transform_static(tf, 'selftest')
        wait(lambda: world._harvest_pub.get_subscription_count() > 0
             and world._center_status_pub.get_subscription_count() > 0, 'Discovery DDS fallita')
        assert manager.target_rejection_reason() is not None
        manager.trigger_callback(Empty())
        assert not manager.started

        point = PointStamped()
        point.header.frame_id = 'table_camera_link_optical'
        point.header.stamp = world.get_clock().now().to_msg()
        point.point.x, point.point.y, point.point.z = .8, .5, 1.2
        world._receive(point)
        wait(lambda: manager.target_rejection_reason() is None, 'Posa valida non accettata')
        assert manager._pear_msg.header.stamp == point.header.stamp
        assert manager._pear_msg.header.frame_id == 'world'
        assert manager._pear_msg.pose.orientation.w == 1.
        expected = world._center_result
        actual = manager._pear_msg.pose.position
        assert (actual.x, actual.y, actual.z) == expected
        manager.trigger_callback(Empty())
        wait(lambda: len(manager.started) == 1, 'Trigger valido non avviato')
        captured = manager.started[0]

        # Later detections update the next target, not the running sequence.
        point.header.stamp = world.get_clock().now().to_msg()
        point.point.x = .9
        world._receive(point)
        wait(lambda: manager.latest_poses is not None
             and manager.latest_poses[1].pose.position.x != expected[0]
             and manager.target_rejection_reason() is None, 'Seconda posa non ricevuta')
        assert captured[1].pose.position.x == expected[0]
        world._invalidate('SOURCE_INVALID', 'selftest')
        wait(lambda: manager._center_status.get('valid') is False, 'Invalidazione non ricevuta')
        manager.trigger_callback(Empty())
        assert len(manager.started) == 1

        manager.center_status_callback(String(data='not json'))
        assert manager.target_rejection_reason() is not None
        print('PICK TARGET PREFLIGHT OK: posa vision, timestamp, trigger valido, invalidazione, snapshot. Nessun movimento.')
    finally:
        executor.shutdown()
        manager.destroy_node()
        world.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

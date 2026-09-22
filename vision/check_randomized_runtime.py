"""Read-only preflight against the live randomized scene; never send motion goals."""
import argparse
import hashlib
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetPositionIK
from rclpy.node import Node
from segmentation_paths import add_segmentation_path

add_segmentation_path()
from orchard_randomization import ArmKinematics, load_scenario, rotation
from pick_place_manager import euler_to_quaternion


def pose_matrix(pose):
    q = pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    t = np.eye(4)
    if norm > 1e-12:
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        t[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    p = pose.position
    t[:3, 3] = [p.x, p.y, p.z]
    return t


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', help='scenario.json printed by the launch')
    parser.add_argument('--max-center-error', type=float, default=.025)
    args = parser.parse_args()
    s = load_scenario(args.scenario)
    world = Path(args.scenario).parent/'orchard.world.sdf'
    assert hashlib.sha256(world.read_bytes()).hexdigest() == s['generated_world_sha256'], 'SDF modificato dopo generazione'
    arm = ArmKinematics(get_package_share_directory('rbt08pkg'))
    rclpy.init(args=[])
    node = Node('randomized_scene_check')
    points, poses = [], []
    subs = [node.create_subscription(PointStamped, '/orchard/vision/center_point_world', points.append, 10),
            node.create_subscription(PoseStamped, '/harvest/pear_pose', poses.append, 10)]
    scene_client = node.create_client(GetPlanningScene, '/get_planning_scene')
    ik_client = node.create_client(GetPositionIK, '/compute_ik')

    def call(client, request):
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('Servizio MoveIt non disponibile: avviare il launch integrato.')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or future.result() is None:
            raise RuntimeError('Timeout servizio MoveIt.')
        return future.result()

    try:
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        response = call(scene_client, req)
        objects = {obj.id: obj for obj in response.scene.world.collision_objects}
        for name in ('pear', 'fruit_branch'):
            assert name in objects, f'{name} mancante: eseguire il test prima del grasp'
            assert objects[name].header.frame_id == 'world', f'Frame collisione inatteso: {name}'

        def primitive_world(obj, i):
            base = pose_matrix(obj.pose) if hasattr(obj, 'pose') else np.eye(4)
            return base @ pose_matrix(obj.primitive_poses[i])

        branch = primitive_world(objects['fruit_branch'], 0)
        np.testing.assert_allclose(branch[:3, 3], s['branch_pose_world'][:3], atol=1e-6)
        np.testing.assert_allclose(branch[:3, :3], rotation(s['branch_pose_world'][3:]), atol=1e-6)
        np.testing.assert_allclose(objects['fruit_branch'].primitives[0].dimensions,
                                   [s['branch_length'], s['branch_radius'] + .015], atol=1e-6)
        np.testing.assert_allclose(primitive_world(objects['pear'], 0)[:3, 3], s['pear_pose_world'][:3], atol=1e-6)
        np.testing.assert_allclose(primitive_world(objects['pear'], 1)[:3, 3],
                                   np.array(s['pear_pose_world'][:3])+[0., 0., .065], atol=1e-6)
        print(f'SCENE ALIGNMENT OK: seed={s["seed"]}, branch and pear aligned; '
              'MoveIt branch includes 15 mm clearance.')

        # These ground-truth goals are validation only, never sent to the manager.
        yaw = math.atan2(s['pear_pose_world'][1], s['pear_pose_world'][0])
        for name, target in arm.targets(s['pear_pose_world'][:3]).items():
            req = GetPositionIK.Request()
            req.ik_request.group_name = 'arm'
            req.ik_request.ik_link_name = 'grasp_frame'
            req.ik_request.pose_stamped.header.frame_id = 'world'
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = target[:3, 3].tolist()
            p.orientation = euler_to_quaternion(0., math.pi/2, yaw)
            req.ik_request.robot_state.is_diff = True
            req.ik_request.robot_state.joint_state.name = s['ik_joint_names']+['left_finger_joint', 'right_finger_joint']
            req.ik_request.robot_state.joint_state.position = s['ik_solutions'][name]+[0., 0.]
            req.ik_request.avoid_collisions = name in ('pre_grasp', 'lift_mid', 'lift')
            req.ik_request.timeout.sec = 3
            result = call(ik_client, req)
            assert result.error_code.val == 1, f'MoveIt IK {name} fallita: {result.error_code.val}; non avviare il ciclo per questo seed.'
        print('MOVEIT IK OK: pre/lift collision-aware; approach/grasp kinematic (fruit contact).')
        # Discard points accumulated during the service calls; wait for fresh data.
        points.clear()
        poses.clear()
        deadline = time.monotonic()+30
        matched = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            for point in reversed(points):
                pose = next((p for p in reversed(poses) if p.header == point.header), None)
                if pose is not None:
                    matched = point, pose
                    break
            if matched is not None:
                break
        assert matched is not None, 'Nessuna coppia recente centro/pear_pose: controllare la pipeline vision.'
        point, pose = matched
        assert point.header.frame_id == 'world'
        xyz = np.array([point.point.x, point.point.y, point.point.z])
        np.testing.assert_allclose(xyz, [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z], atol=1e-9)
        error = np.linalg.norm(xyz-np.array(s['pear_pose_world'][:3]))
        assert np.isfinite(error) and error <= args.max_center_error, f'Errore centro {error:.4f} m oltre soglia {args.max_center_error} m'
        print(f'VISION OK: centre error={error*1000:.1f} mm. No robot motion commanded.')
        print('RANDOMIZED PREFLIGHT OK. La traiettoria completa e la presa fisica richiedono ancora il test Gazebo.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

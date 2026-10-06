"""Read-only preflight against the live randomized scene; never send motion goals."""
import argparse
import copy
import hashlib
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetPositionIK, GetStateValidity
from rclpy.node import Node
from segmentation_paths import add_segmentation_path

add_segmentation_path()
from orchard_randomization import ArmKinematics, load_scenario, rotation
from pick_place_manager import euler_to_quaternion, multiply_quaternions


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


def checked_ik(request, previous, call_ik, call_validity, name):
    """Try local continuation before the independent numerical seed.

    Collision-unchecked queries are diagnostic only and never count as success.
    Passing this check does not certify a continuous Cartesian trajectory.
    """
    original = copy.deepcopy(request)
    seeds = [original.ik_request.robot_state]
    if previous is not None:
        seeds.insert(0, previous)
    errors = []
    for seed in seeds:
        candidate = copy.deepcopy(original)
        candidate.ik_request.robot_state = copy.deepcopy(seed)
        result = call_ik(candidate)
        if result.error_code.val == 1:
            print(f'  IK {name}: OK')
            return result.solution
        errors.append(result.error_code.val)

    details = []
    if original.ik_request.avoid_collisions:
        # Inspect both the original numerical state and any unconstrained IK
        # results; report actual MoveIt contacts rather than guessing a cause.
        states = [('manifest seed', original.ik_request.robot_state)]
        for seed in seeds:
            diagnostic = copy.deepcopy(original)
            diagnostic.ik_request.robot_state = copy.deepcopy(seed)
            diagnostic.ik_request.avoid_collisions = False
            result = call_ik(diagnostic)
            if result.error_code.val == 1:
                states.append(('kinematic IK solution', result.solution))
            else:
                details.append(f'kinematic IK also failed ({result.error_code.val})')
        for label, state in states:
            validity = GetStateValidity.Request()
            validity.robot_state = copy.deepcopy(state)
            validity.group_name = original.ik_request.group_name
            report = call_validity(validity)
            pairs = sorted({f'{c.contact_body_1} <-> {c.contact_body_2}' for c in report.contacts})
            details.append(f'{label}: valid={report.valid}; contacts={pairs or "none reported"}')
        if any(label == 'kinematic IK solution' for label, _ in states):
            details.insert(0, 'kinematic IK exists, but collision-aware search failed')
    raise RuntimeError(
        f'MoveIt IK {name} failed; checked attempts={errors}. '
        + ' | '.join(details)
        + '. Do not trigger this seed. No collision-unchecked result was accepted.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', help='scenario.json printed by the launch')
    parser.add_argument('--max-center-error', type=float, default=.025)
    args = parser.parse_args()
    s = load_scenario(args.scenario)
    world = Path(args.scenario).parent/'orchard.world.sdf'
    assert hashlib.sha256(world.read_bytes()).hexdigest() == s['generated_world_sha256'], 'SDF modified after generation'
    arm = ArmKinematics(get_package_share_directory('rbt08pkg'))
    rclpy.init(args=[])
    node = Node('randomized_scene_check')
    points, poses, grasp_poses, pre_poses = [], [], [], []
    subs = [node.create_subscription(PointStamped, '/orchard/vision/center_point_world', points.append, 10),
            node.create_subscription(PoseStamped, '/harvest/pear_pose', poses.append, 10),
            node.create_subscription(PoseStamped, '/harvest/grasp_pose', grasp_poses.append, 10),
            node.create_subscription(PoseStamped, '/harvest/pre_grasp_pose', pre_poses.append, 10)]
    scene_client = node.create_client(GetPlanningScene, '/get_planning_scene')
    ik_client = node.create_client(GetPositionIK, '/compute_ik')
    validity_client = node.create_client(GetStateValidity, '/check_state_validity')

    def call(client, request):
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('MoveIt service unavailable: start the integrated launch.')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        if not future.done() or future.result() is None:
            raise RuntimeError('MoveIt service timeout.')
        return future.result()

    try:
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        response = call(scene_client, req)
        objects = {obj.id: obj for obj in response.scene.world.collision_objects}
        for name in ('pear', 'fruit_branch'):
            assert name in objects, f'{name} missing: run this check before grasping'
            assert objects[name].header.frame_id == 'world', f'Unexpected collision frame: {name}'

        def primitive_world(obj, i):
            base = pose_matrix(obj.pose) if hasattr(obj, 'pose') else np.eye(4)
            return base @ pose_matrix(obj.primitive_poses[i])

        branch = primitive_world(objects['fruit_branch'], 0)
        np.testing.assert_allclose(branch[:3, 3], s['branch_pose_world'][:3], atol=1e-6)
        np.testing.assert_allclose(branch[:3, :3], rotation(s['branch_pose_world'][3:]), atol=1e-6)
        np.testing.assert_allclose(objects['fruit_branch'].primitives[0].dimensions,
                                   [s['branch_length'], s['branch_radius'] + .015], atol=1e-6)
        np.testing.assert_allclose(primitive_world(objects['pear'], 0)[:3, 3], s['pear_pose_world'][:3], atol=1e-6)
        np.testing.assert_allclose(primitive_world(objects['pear'], 0)[:3, :3],
                                   rotation(s['pear_pose_world'][3:]), atol=1e-6)
        np.testing.assert_allclose(primitive_world(objects['pear'], 1)[:3, 3],
                                   np.array(s['pear_pose_world'][:3]) +
                                   rotation(s['pear_pose_world'][3:]) @ [0., 0., .065], atol=1e-6)
        print(f'SCENE ALIGNMENT OK: seed={s["seed"]}, branch and pear aligned; '
              'MoveIt branch includes 15 mm clearance.')

        # These ground-truth goals are validation only, never sent to the manager.
        approach_yaw = s.get('grasp_approach_yaw_world')
        yaw = approach_yaw if approach_yaw is not None else math.atan2(s['pear_pose_world'][1], s['pear_pose_world'][0])
        previous = None
        pear_rpy = s['pear_pose_world'][3:]
        for name, target in arm.targets(s['pear_pose_world'][:3], approach_yaw, pear_rpy).items():
            req = GetPositionIK.Request()
            req.ik_request.group_name = 'arm'
            req.ik_request.ik_link_name = 'grasp_frame'
            req.ik_request.pose_stamped.header.frame_id = 'world'
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = target[:3, 3].tolist()
            if approach_yaw is None:
                p.orientation = euler_to_quaternion(0., math.pi/2, yaw)
            else:
                p.orientation = multiply_quaternions(
                    euler_to_quaternion(*pear_rpy),
                    euler_to_quaternion(0., math.pi/2, yaw-pear_rpy[2]))
            req.ik_request.robot_state.is_diff = True
            req.ik_request.robot_state.joint_state.name = s['ik_joint_names']+['left_finger_joint', 'right_finger_joint']
            req.ik_request.robot_state.joint_state.position = s['ik_solutions'][name]+[0., 0.]
            req.ik_request.avoid_collisions = name in ('pre_grasp', 'lift_mid', 'lift')
            req.ik_request.timeout.sec = 3
            previous = checked_ik(
                req, previous, lambda request: call(ik_client, request),
                lambda request: call(validity_client, request), name)
        print('MOVEIT IK OK: pre/lift collision-aware; approach/grasp kinematic (fruit contact).')
        # Discard points accumulated during the service calls; wait for fresh data.
        points.clear()
        poses.clear()
        grasp_poses.clear()
        pre_poses.clear()
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
        assert matched is not None, 'No recent center/pear_pose pair: check the vision pipeline.'
        point, pose = matched
        assert point.header.frame_id == 'world'
        xyz = np.array([point.point.x, point.point.y, point.point.z])
        np.testing.assert_allclose(xyz, [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z], atol=1e-9)
        np.testing.assert_allclose(pose_matrix(pose.pose)[:3, :3],
                                   rotation(s['pear_pose_world'][3:]), atol=1e-6,
                                   err_msg='Published pear frame does not match the generated world')
        error = np.linalg.norm(xyz-np.array(s['pear_pose_world'][:3]))
        assert np.isfinite(error) and error <= args.max_center_error, f'Center error {error:.4f} m exceeds threshold {args.max_center_error} m'
        print(f'VISION OK: centre error={error*1000:.1f} mm. No robot motion commanded.')
        # Verify the running manager, not just the offline scenario calculation.
        deadline = time.monotonic()+30
        pair = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            for grasp_msg in reversed(grasp_poses):
                pre_msg = next((p for p in reversed(pre_poses) if p.header == grasp_msg.header), None)
                if grasp_msg.header.frame_id != 'base_link':
                    continue
                candidate_xyz = (arm.base @ pose_matrix(grasp_msg.pose))[:3, 3]
                has_vision_position = any(
                    np.linalg.norm(candidate_xyz-np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z])) < 1e-6
                    for p in poses if p.header.frame_id == 'world')
                if pre_msg is not None and has_vision_position:
                    pair = grasp_msg, pre_msg
                    break
            if pair is not None:
                break
        assert pair is not None, 'No manager grasp/pre-grasp pair matching a recent vision position.'
        grasp_msg, pre_msg = pair
        assert grasp_msg.header.frame_id == 'base_link', 'Unexpected manager target frame'
        grasp_world, pre_world = arm.base @ pose_matrix(grasp_msg.pose), arm.base @ pose_matrix(pre_msg.pose)
        measured = grasp_world[:3, 3]
        assert any(np.linalg.norm(measured-np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z])) < 1e-6
                   for p in poses if p.header.frame_id == 'world'), 'Manager target does not match a received vision position'
        expected = arm.targets(measured, approach_yaw, pear_rpy)
        np.testing.assert_allclose(grasp_world, expected['grasp'], atol=1e-6,
                                   err_msg='Manager orientation differs from scenario; check launch mode and rebuilt files')
        np.testing.assert_allclose(pre_world, expected['pre_grasp'], atol=1e-6,
                                   err_msg='Pre-grasp does not follow the selected approach axis')
        print(f'GRASP ORIENTATION OK: mode={s.get("grasp_orientation", "radial")}; '
              'live target uses vision position and matching pre-grasp direction.')
        print('RANDOMIZED PREFLIGHT OK. The complete trajectory and physical grasp still require a Gazebo test.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

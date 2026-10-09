"""Offline tests of the actual trigger guard; no ROS installation required."""
import ast
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import time
import unittest
from segmentation_paths import add_segmentation_path
add_segmentation_path()
from grasp_geometry import quaternion_rotation, robot_side_grasp_offset


root = Path(__file__).resolve().parents[1]
source = root / 'rbt08pkg/scripts/pick_place_manager.py'
if not source.exists():
    source = root / 'src/rbt08pkg/scripts/pick_place_manager.py'
tree = ast.parse(source.read_text(encoding='utf-8'))
manager = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'PickPlaceManager')
methods = [n for n in manager.body if isinstance(n, ast.FunctionDef)
           and n.name in ('target_rejection_reason', 'center_status_callback')]
scope = {'math': math, 'time': time, 'json': json, 'String': object}
exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), 'exec'), scope)


class TriggerChecks(unittest.TestCase):
    def setUp(self):
        self.node = NS(
            _pear_msg=NS(header=NS(stamp=NS(sec=100, nanosec=0)),
                         pose=NS(position=NS(x=.85, y=.55, z=1.3))),
            latest_poses=(object(),), _pear_received_at=time.monotonic(),
            _require_vision_status=True, _max_pose_age=5.,
            _center_status={'valid': True, 'stamp': {'sec': 100, 'nanosec': 0}, 'received_age_s': .1},
            _center_status_at=time.monotonic(),
            get_clock=lambda: NS(now=lambda: NS(nanoseconds=100_500_000_000)),
        )

    def reason(self):
        return scope['target_rejection_reason'](self.node)

    def test_valid_pose(self):
        self.assertIsNone(self.reason())

    def test_missing_pose(self):
        self.node._pear_msg = None
        self.assertIsNotNone(self.reason())

    def test_missing_transform(self):
        self.node.latest_poses = None
        self.assertIsNotNone(self.reason())

    def test_nonfinite_coordinates(self):
        for value in (math.nan, math.inf, -math.inf):
            self.node._pear_msg.pose.position.x = value
            self.assertIsNotNone(self.reason())

    def test_invalid_or_other_measurement(self):
        for status in ({}, {'valid': False},
                       {'valid': True, 'stamp': {'sec': 99, 'nanosec': 0}}):
            self.node._center_status = status
            self.assertIsNotNone(self.reason())

    def test_expired_receipt_even_with_paused_simulation(self):
        self.node._pear_received_at -= 6
        self.assertIsNotNone(self.reason())

    def test_delayed_measurement(self):
        self.node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=106_000_000_000))
        self.assertIsNotNone(self.reason())

    def test_future_stamp_after_clock_reset(self):
        self.node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=1_000_000_000))
        self.assertIsNotNone(self.reason())

    def test_old_source_not_refreshed_by_status_heartbeat(self):
        self.node._center_status['received_age_s'] = 6.
        self.assertIsNotNone(self.reason())

    def test_invalid_age(self):
        for age in (None, 'invalid', math.nan, math.inf):
            self.node._center_status['received_age_s'] = age
            self.assertIsNotNone(self.reason())

    def test_malformed_status_invalidates_previous(self):
        for data in ('{', '[]', 'null'):
            scope['center_status_callback'](self.node, NS(data=data))
            self.assertIsNotNone(self.reason())

    def test_static_mode_needs_no_center_status(self):
        self.node._require_vision_status = False
        self.node._center_status = {}
        self.assertIsNone(self.reason())


class GraspAttitudeChecks(unittest.TestCase):
    """Exercise the actual pose callback with lightweight ROS message/TF doubles."""

    def setUp(self):
        def pose(**kwargs):
            return NS(position=kwargs.get('position', NS(x=0., y=0., z=0.)),
                      orientation=kwargs.get('orientation', NS(x=0., y=0., z=0., w=1.)))

        def stamped(**kwargs):
            return NS(header=kwargs.get('header', NS(frame_id='world', stamp=0)),
                      pose=kwargs.get('pose', pose()))

        def apply_tf(msg, tf):
            result = copy.deepcopy(msg)
            p, q = msg.pose.position, msg.pose.orientation
            c, s = math.cos(tf.angle), math.sin(tf.angle)
            result.pose.position = NS(x=c*p.x-s*p.y+.1, y=s*p.x+c*p.y-.2, z=p.z)
            c, s = math.cos(tf.angle/2), math.sin(tf.angle/2)
            result.pose.orientation = NS(x=c*q.x-s*q.y, y=c*q.y+s*q.x,
                                         z=c*q.z+s*q.w, w=c*q.w-s*q.z)
            result.header.frame_id = 'base_link'
            return result

        self.stamped, self.apply_tf = stamped, apply_tf
        funcs = [n for n in tree.body if isinstance(n, ast.FunctionDef)
                 and n.name in ('euler_to_quaternion', 'multiply_quaternions')]
        funcs += [n for n in manager.body if isinstance(n, ast.FunctionDef) and n.name == 'pear_pose_callback']
        self.scope = dict(math=math, time=time, PoseStamped=stamped, Pose=pose, Point=NS, Quaternion=NS,
                          quaternion_rotation=quaternion_rotation, robot_side_grasp_offset=robot_side_grasp_offset,
                          do_transform_pose_stamped=apply_tf,
                          rclpy=NS(time=NS(Time=lambda: 0), duration=NS(Duration=lambda **kw: 0)))
        exec(compile(ast.Module(body=funcs, type_ignores=[]), str(source), 'exec'), self.scope)
        self.tf = NS(angle=-.4)
        self.node = NS(_grasp_orientation='branch',
                       tf_buffer=NS(lookup_transform=lambda *a, **kw: self.tf),
                       get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: 1)),
                       get_logger=lambda: NS(warn=lambda msg: None), publish_all_markers=lambda *a: None)
        for name in ('pub_pre_grasp', 'pub_grasp', 'pub_lift'):
            setattr(self.node, name, NS(publish=lambda msg: None))

    def update(self, xyz, pitch=0.):
        msg = self.stamped()
        msg.pose.position = NS(x=xyz[0], y=xyz[1], z=xyz[2])
        msg.pose.orientation = self.scope['euler_to_quaternion'](0., pitch, .2)
        self.scope['pear_pose_callback'](self.node, msg)
        return self.node.latest_poses

    def test_branch_pitch_tilts_grasp_and_pre_grasp_axis(self):
        pre, grasp, *_ = self.update((.85, .55, 1.3), pitch=.08)
        q = grasp.pose.orientation
        axis = (2*(q.x*q.z+q.w*q.y), 2*(q.y*q.z-q.w*q.x), 1-2*(q.x*q.x+q.y*q.y))
        self.assertGreater(abs(axis[2]), .05)
        for key, direction in zip(('x', 'y', 'z'), axis):
            self.assertAlmostEqual(getattr(grasp.pose.position, key)-getattr(pre.pose.position, key), .25*direction)
        pear_in_base = self.apply_tf(self.node._pear_msg, self.tf)
        pq = pear_in_base.pose.orientation
        p = pear_in_base.pose.position
        offset = robot_side_grasp_offset((p.x, p.y, p.z), quaternion_rotation((pq.x, pq.y, pq.z, pq.w)))
        expected = self.scope['multiply_quaternions'](
            pear_in_base.pose.orientation,
            self.scope['euler_to_quaternion'](0., math.pi/2, offset))
        for key in ('x', 'y', 'z', 'w'):
            self.assertAlmostEqual(getattr(q, key), getattr(expected, key))

    def test_branch_composes_observed_center_known_pear_yaw_and_relative_grasp(self):
        pre, grasp, lift, *_ = self.update((.85, .55, 1.3))
        q = grasp.pose.orientation
        axis = (2*(q.x*q.z+q.w*q.y), 2*(q.y*q.z-q.w*q.x), 1-2*(q.x*q.x+q.y*q.y))
        expected = self.apply_tf(self.node._pear_msg, self.tf).pose.position
        expected_yaw = math.atan2(expected.y, expected.x)
        self.assertEqual(grasp.pose.position, expected)
        for value, target in zip(axis, (math.cos(expected_yaw), math.sin(expected_yaw), 0)):
            self.assertAlmostEqual(value, target)
        for key, direction in zip(('x', 'y', 'z'), axis):
            self.assertAlmostEqual(getattr(grasp.pose.position, key)-getattr(pre.pose.position, key), .25*direction)
        self.assertEqual(pre.pose.orientation, grasp.pose.orientation)
        self.assertEqual(lift.pose.orientation, grasp.pose.orientation)
        self.assertAlmostEqual(lift.pose.position.z-pre.pose.position.z, .1)
        other = self.update((.83, .57, 1.32))[1]
        self.assertNotEqual(other.pose.orientation, grasp.pose.orientation)
        self.assertNotEqual(other.pose.position, grasp.pose.position)

    def test_degenerate_projection_invalidates_previous_target(self):
        self.update((.85, .55, 1.3))
        # Invert the test TF translation: the new point lies on pear +Z.
        c, s = math.cos(self.tf.angle), math.sin(self.tf.angle)
        self.update((c*(-.1)+s*.2, -s*(-.1)+c*.2, 1.3))
        self.assertIsNone(self.node.latest_poses)

    def test_projection_matches_robot_side_for_yaws_and_tilt(self):
        for yaw in (math.radians(-15), math.radians(40)):
            q = self.scope['euler_to_quaternion'](0., math.radians(20), yaw)
            r = quaternion_rotation((q.x, q.y, q.z, q.w))
            p = (.85, .55, 1.3)
            offset = robot_side_grasp_offset(p, r)
            d = [sum(r[i][j]*v for j, v in enumerate((math.cos(offset), math.sin(offset), 0.))) for i in range(3)]
            normal = [row[2] for row in r]
            projection = [p[i]-sum(p[j]*normal[j] for j in range(3))*normal[i] for i in range(3)]
            norm = math.sqrt(sum(v*v for v in projection))
            for a, b in zip(d, projection):
                self.assertAlmostEqual(a, b/norm)

    def test_radial_mode_keeps_robot_to_pear_direction(self):
        self.node._grasp_orientation = 'radial'
        pre, grasp, lift, *_ = self.update((.85, .55, 1.3))
        p = grasp.pose.position
        distance = math.hypot(p.x, p.y)
        self.assertAlmostEqual(p.x-pre.pose.position.x, .25*p.x/distance)
        self.assertAlmostEqual(p.y-pre.pose.position.y, .25*p.y/distance)
        self.assertAlmostEqual(lift.pose.position.z-pre.pose.position.z, .2)

    def test_missing_pear_tf_invalidates_target(self):
        self.update((.85, .55, 1.3))
        def lookup(target, origin, *args, **kwargs):
            raise RuntimeError('Missing pear TF')
        self.node.tf_buffer.lookup_transform = lookup
        msg = self.stamped()
        msg.header.frame_id = 'camera'
        msg.pose.position = NS(x=.85, y=.55, z=1.3)
        self.scope['pear_pose_callback'](self.node, msg)
        self.assertIsNone(self.node.latest_poses)


class PreflightIKChecks(unittest.TestCase):
    def setUp(self):
        checker = Path(__file__).with_name('check_randomized_runtime.py')
        checker_tree = ast.parse(checker.read_text(encoding='utf-8'))
        funcs = [n for n in checker_tree.body if isinstance(n, ast.FunctionDef) and n.name == 'checked_ik']
        namespace = dict(copy=copy, GetStateValidity=NS(Request=lambda: NS()), print=lambda *a: None)
        exec(compile(ast.Module(body=funcs, type_ignores=[]), str(checker), 'exec'), namespace)
        self.solve = namespace['checked_ik']
        self.request = NS(ik_request=NS(robot_state=NS(name='manifest'), avoid_collisions=True, group_name='arm'))

    def test_previous_solution_is_tried_then_manifest_with_collision_checks(self):
        attempts = []
        def ik(request):
            attempts.append((request.ik_request.robot_state.name, request.ik_request.avoid_collisions))
            return NS(error_code=NS(val=-31 if len(attempts) == 1 else 1), solution=NS(name='solved'))
        result = self.solve(self.request, NS(name='previous'), ik, lambda _: self.fail('Unexpected diagnostic'), 'lift')
        self.assertEqual(result.name, 'solved')
        self.assertEqual(attempts, [('previous', True), ('manifest', True)])
        self.assertEqual(self.request.ik_request.robot_state.name, 'manifest')

    def test_kinematic_success_cannot_bypass_collision_failure(self):
        def ik(request):
            return NS(error_code=NS(val=-31 if request.ik_request.avoid_collisions else 1), solution=NS(name='colliding'))
        validity = lambda _: NS(valid=False, contacts=[NS(contact_body_1='link6', contact_body_2='ground')])
        with self.assertRaisesRegex(RuntimeError, 'link6 <-> ground'):
            self.solve(self.request, None, ik, validity, 'lift')
        self.assertTrue(self.request.ik_request.avoid_collisions)

    def test_unreachable_pose_reports_kinematic_failure(self):
        ik = lambda _: NS(error_code=NS(val=-31), solution=None)
        validity = lambda _: NS(valid=True, contacts=[])
        with self.assertRaisesRegex(RuntimeError, 'kinematic IK also failed'):
            self.solve(self.request, None, ik, validity, 'lift')


if __name__ == '__main__':
    unittest.main()

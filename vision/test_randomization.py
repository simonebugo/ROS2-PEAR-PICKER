"""Offline geometry, determinism and actual FK checks. Requires only NumPy."""
import copy
import json
import itertools
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from segmentation_paths import add_segmentation_path

add_segmentation_path()
from orchard_randomization import ArmKinematics, generate_scenario, load_bounds, load_scenario, reference_approach_yaw, rotation, rotation_error, write_scenario


ROOT = Path(__file__).resolve().parents[1]
SHARE = ROOT/'rbt08pkg'
if not SHARE.exists():
    SHARE = ROOT/'src/rbt08pkg'
if not SHARE.exists():
    from ament_index_python.packages import get_package_share_directory
    SHARE = Path(get_package_share_directory('rbt08pkg'))
BOUNDS = SHARE/'config/orchard_randomization.json'


class RandomizationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.arm = ArmKinematics(SHARE)
        cls.scenarios = [generate_scenario(SHARE, BOUNDS, seed) for seed in range(12)]

    def test_same_seed_reproduces_all_parameters_and_ik(self):
        self.assertEqual(self.scenarios[3], generate_scenario(SHARE, BOUNDS, 3))

    def test_continuous_samples_and_bounds(self):
        self.assertEqual(len({tuple(s['pear_pose_world']) for s in self.scenarios}), 12)
        for scenario in self.scenarios:
            for name, value in scenario['sampled'].items():
                lo, hi = scenario['bounds'][name]
                self.assertLessEqual(lo, value)
                self.assertLessEqual(value, hi)
            self.assertLess(scenario['pear_pose_world'][0], 1.15)

    def test_branch_pivot_and_stem_stay_connected(self):
        for s in self.scenarios:
            center = np.array(s['branch_pose_world'][:3])
            direction = rotation(s['branch_pose_world'][3:])[:, 2]
            pivot = center-s['branch_length']/2*direction
            np.testing.assert_allclose(pivot, [1.35, .55, s['sampled']['branch_height_m']], atol=1e-12)
            attached = pivot+s['sampled']['pear_distance_m']*direction
            np.testing.assert_allclose(np.array(s['pear_pose_world'][:3])+[0, 0, .18], attached, atol=1e-12)

    def test_ik_solutions_reach_each_goal_with_joint_limits(self):
        for scenario in self.scenarios:
            targets = self.arm.targets(scenario['pear_pose_world'][:3], scenario['grasp_approach_yaw_world'])
            for name, q in scenario['ik_solutions'].items():
                self.assertTrue(np.all(q >= self.arm.lower) and np.all(q <= self.arm.upper))
                actual, _ = self.arm.forward(q)
                self.assertLess(np.linalg.norm(actual[:3, 3]-targets[name][:3, 3]), .0005)
                self.assertLess(np.linalg.norm(rotation_error(targets[name][:3, :3], actual[:3, :3])), .003)

    def test_branch_yaw_rotates_attitude_and_retreat_together(self):
        nominal = reference_approach_yaw(SHARE)
        for s in self.scenarios:
            yaw = s['grasp_approach_yaw_world']
            self.assertAlmostEqual(yaw-nominal, np.deg2rad(s['sampled']['branch_yaw_deg']))
            goals = self.arm.targets(s['pear_pose_world'][:3], yaw)
            grasp, pre = goals['grasp'], goals['pre_grasp']
            np.testing.assert_allclose(grasp[:3, :3], rotation((0, np.pi/2, yaw)), atol=1e-12)
            np.testing.assert_allclose(grasp[:3, 3]-pre[:3, 3], .25*grasp[:3, 2], atol=1e-12)
            np.testing.assert_allclose(goals['lift'][:3, 3]-pre[:3, 3], [0, 0, .1], atol=1e-12)
            np.testing.assert_allclose(s['pear_pose_world'][3:5], [0, 0])
            self.assertAlmostEqual(s['pear_pose_world'][5], np.deg2rad(s['sampled']['branch_yaw_deg']))
            np.testing.assert_allclose(
                grasp[:3, :3],
                rotation(s['pear_pose_world'][3:]) @ rotation((0, np.pi/2, nominal)),
                atol=1e-12)

    def test_branch_attitude_is_independent_of_measured_position(self):
        yaw = reference_approach_yaw(SHARE)+.2
        a = self.arm.targets([.85, .55, 1.3], yaw)['grasp']
        b = self.arm.targets([.83, .57, 1.32], yaw)['grasp']
        np.testing.assert_allclose(a[:3, :3], b[:3, :3])
        np.testing.assert_allclose(b[:3, 3], [.83, .57, 1.32])

    def test_radial_mode_preserves_previous_geometry(self):
        s = generate_scenario(SHARE, BOUNDS, 42, 'radial')
        self.assertIsNone(s['grasp_approach_yaw_world'])
        p = np.array(s['pear_pose_world'][:3])
        goals = self.arm.targets(p)
        np.testing.assert_allclose(goals['lift'][:3, 3]-goals['pre_grasp'][:3, 3], [0, 0, .2])
        np.testing.assert_allclose(goals['grasp'][:3, 2], [*list(p[:2]/np.linalg.norm(p[:2])), 0], atol=1e-12)
        nominal = self.arm.targets([.85, .55, 1.3])
        branch = self.arm.targets([.85, .55, 1.3], reference_approach_yaw(SHARE))
        for key in ('pre_grasp', 'approach_mid', 'grasp'):
            np.testing.assert_allclose(nominal[key], branch[key], atol=1e-12)

    def test_wide_and_edge_oriented_targets_are_reachable(self):
        for profile in ('wide', 'edge'):
            for seed in (7, 42, 123):
                s = generate_scenario(SHARE, SHARE/f'config/orchard_randomization_{profile}.json', seed)
                goals = self.arm.targets(s['pear_pose_world'][:3], s['grasp_approach_yaw_world'])
                for name, q in s['ik_solutions'].items():
                    actual, _ = self.arm.forward(q)
                    self.assertLess(np.linalg.norm(actual[:3, 3]-goals[name][:3, 3]), .0005)
                    self.assertLess(np.linalg.norm(rotation_error(goals[name][:3, :3], actual[:3, :3])), .003)

    def test_jacobian_matches_finite_difference(self):
        q = np.array([.6, -.4, .5, .2, 1.2, -.9])
        actual, jac = self.arm.forward(q)
        for i in range(6):
            delta = q.copy()
            delta[i] += 1e-6
            shifted, _ = self.arm.forward(delta)
            numeric = np.r_[(shifted[:3, 3]-actual[:3, 3])/1e-6,
                            rotation_error(shifted[:3, :3], actual[:3, :3])/1e-6]
            np.testing.assert_allclose(jac[:, i], numeric, atol=1e-6)

    def test_sdf_and_manifest_agree_and_template_is_unchanged(self):
        before = (SHARE/'worlds/orchard.world.sdf').read_bytes()
        with tempfile.TemporaryDirectory() as directory:
            world, manifest = write_scenario(SHARE, self.scenarios[0], directory)
            s = load_scenario(manifest)
            root = ET.parse(world)
            pose = lambda path: [float(v) for v in root.findtext(path).split()]
            np.testing.assert_allclose(pose(".//model[@name='pear']/pose"), s['pear_pose_world'], atol=1e-14)
            for kind in ('collision', 'visual'):
                np.testing.assert_allclose(pose(f".//model[@name='pear_tree']/link/{kind}[@name='fruit_branch_{kind}']/pose"), s['branch_pose_tree'], atol=1e-14)
            self.assertEqual(root.findtext(".//model[@name='pear_tree']/plugin/child_model"), 'pear')
        self.assertEqual(before, (SHARE/'worlds/orchard.world.sdf').read_bytes())

    def test_unreachable_target_has_no_solution(self):
        target = np.eye(4)
        target[:3, 3] = [10, 10, 10]
        self.assertIsNone(self.arm.solve(target))

    def test_invalid_bounds_and_out_of_frame_fail_explicitly(self):
        data = load_bounds(BOUNDS)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'bounds.json'
            bad = copy.deepcopy(data)
            bad['branch_height_m'] = [2, 1]
            path.write_text(json.dumps(bad), encoding='utf-8')
            with self.assertRaises(ValueError):
                load_bounds(path)
            data['branch_height_m'], data['max_attempts'] = [10, 11], 2
            path.write_text(json.dumps(data), encoding='utf-8')
            with self.assertRaisesRegex(RuntimeError, 'No valid scenario'):
                generate_scenario(SHARE, path, 42)

    def test_legacy_radial_bound_corners_still_pass(self):
        data = load_bounds(BOUNDS)
        names = ('branch_height_m', 'branch_yaw_deg', 'branch_elevation_deg', 'pear_distance_m')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'bounds.json'
            for values in itertools.product(*(data[name] for name in names)):
                corner = dict(data, max_attempts=1)
                corner.update({name: [v, v] for name, v in zip(names, values)})
                path.write_text(json.dumps(corner), encoding='utf-8')
                scenario = generate_scenario(SHARE, path, 0, 'radial')
                self.assertEqual(scenario['attempt'], 1)

    def test_branch_corner_without_ik_is_rejected_explicitly(self):
        # A reachable point with radial attitude need not be reachable with
        # the requested branch attitude. Never silently fall back to radial.
        data = load_bounds(BOUNDS)
        data.update(branch_height_m=[1.44, 1.44], branch_yaw_deg=[-12., -12.],
                    branch_elevation_deg=[-5., -5.], pear_distance_m=[.44, .44], max_attempts=1)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'bounds.json'
            path.write_text(json.dumps(data), encoding='utf-8')
            with self.assertRaisesRegex(RuntimeError, 'ik_pre_grasp'):
                generate_scenario(SHARE, path, 0, 'branch')


if __name__ == '__main__':
    unittest.main()

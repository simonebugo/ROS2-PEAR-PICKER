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
from orchard_randomization import ArmKinematics, generate_scenario, load_bounds, load_scenario, rotation, rotation_error, write_scenario


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
            targets = self.arm.targets(scenario['pear_pose_world'][:3])
            for name, q in scenario['ik_solutions'].items():
                self.assertTrue(np.all(q >= self.arm.lower) and np.all(q <= self.arm.upper))
                actual, _ = self.arm.forward(q)
                self.assertLess(np.linalg.norm(actual[:3, 3]-targets[name][:3, 3]), .0005)
                self.assertLess(np.linalg.norm(rotation_error(targets[name][:3, :3], actual[:3, :3])), .003)

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
            with self.assertRaisesRegex(RuntimeError, 'Nessuno scenario valido'):
                generate_scenario(SHARE, path, 42)

    def test_default_bound_corners_pass_camera_and_ik_filters(self):
        data = load_bounds(BOUNDS)
        names = ('branch_height_m', 'branch_yaw_deg', 'branch_elevation_deg', 'pear_distance_m')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'bounds.json'
            for values in itertools.product(*(data[name] for name in names)):
                corner = dict(data, max_attempts=1)
                corner.update({name: [v, v] for name, v in zip(names, values)})
                path.write_text(json.dumps(corner), encoding='utf-8')
                scenario = generate_scenario(SHARE, path, 0)
                self.assertEqual(scenario['attempt'], 1)


if __name__ == '__main__':
    unittest.main()

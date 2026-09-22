"""Offline regression tests for collision permissions, incomplete paths and IK jumps."""
import ast
import copy
import threading
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

from segmentation_paths import add_segmentation_path
add_segmentation_path()
from trajectory_checks import cartesian_problem


class CartesianChecks(unittest.TestCase):
    def setUp(self):
        self.state = NS(name=['j1', 'j2'], position=[0., 0.])
        self.path = NS(joint_names=['j1', 'j2'], points=[NS(positions=[0., 0.]), NS(positions=[.05, .1])])
        self.result = NS(error_code=NS(val=1), fraction=1., solution=NS(joint_trajectory=self.path))

    def test_complete_smooth_path(self):
        self.assertIsNone(cartesian_problem(self.result, self.state))

    def test_partial_path_is_never_executed(self):
        for fraction in (.5, .9, .99, float('nan')):
            self.result.fraction = fraction
            self.assertIsNotNone(cartesian_problem(self.result, self.state))

    def test_joint_branch_jump(self):
        self.path.points[-1].positions = [2., .1]
        self.assertIn('Salto IK', cartesian_problem(self.result, self.state))

    def test_jump_from_actual_start_is_checked(self):
        self.state.position = [1., 0.]
        self.assertIn('Salto IK', cartesian_problem(self.result, self.state))

    def test_failed_empty_or_nonfinite_result(self):
        self.result.error_code.val = -1
        self.assertIsNotNone(cartesian_problem(self.result, self.state))
        self.result.error_code.val = 1
        self.path.points[-1].positions = [float('inf'), .1]
        self.assertIsNotNone(cartesian_problem(self.result, self.state))
        self.path.points = []
        self.assertIsNotNone(cartesian_problem(self.result, self.state))

    def test_permissions_only_allow_pear_gripper_and_preserve_ground(self):
        root = Path(__file__).resolve().parents[1]
        source = root/'rbt08pkg/scripts/pick_place_manager.py'
        if not source.exists():
            source = root/'src/rbt08pkg/scripts/pick_place_manager.py'
        tree = ast.parse(source.read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'PickPlaceManager')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'allow_fruit_contact')
        scope = {'copy': copy, 'PlanningSceneComponents': NS(ALLOWED_COLLISION_MATRIX=1), 'AllowedCollisionEntry': NS}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), scope)
        acm = NS(entry_names=['ground', 'link1'], entry_values=[NS(enabled=[False, False]), NS(enabled=[False, False])])
        initial = copy.deepcopy(acm)
        node = NS(read_scene=lambda _: NS(allowed_collision_matrix=acm), apply_acm=lambda _: True)
        self.assertTrue(scope['allow_fruit_contact'](node))
        self.assertEqual(node._saved_acm, initial)
        for i, row in enumerate(acm.entry_values):
            for j, allowed in enumerate(row.enabled):
                pair = {acm.entry_names[i], acm.entry_names[j]}
                expected = 'pear' in pair and bool(pair & {'left_finger_link', 'right_finger_link', 'gripper_base_link'})
                self.assertEqual(allowed, expected)

    def test_wait_for_stable_arm_rejects_drift_and_stale_samples(self):
        root = Path(__file__).resolve().parents[1]
        source = root/'rbt08pkg/scripts/pick_place_manager.py'
        if not source.exists():
            source = root/'src/rbt08pkg/scripts/pick_place_manager.py'
        cls = next(n for n in ast.parse(source.read_text(encoding='utf-8')).body
                   if isinstance(n, ast.ClassDef) and n.name == 'PickPlaceManager')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'wait_for_stable_arm')
        for mode in ('stable', 'drifting', 'stale'):
            clock = NS(value=0.)
            node = NS(_joint_lock=threading.Lock(), _joint_state=NS(
                name=[f'j{i}' for i in range(1, 7)], position=[0.]*6),
                _joint_received_at=0., _joint_count=0,
                get_logger=lambda: NS(error=lambda _: None))
            def advance(seconds):
                clock.value += seconds
                if mode != 'stale':
                    node._joint_received_at = clock.value
                    node._joint_count += 1
                    if mode == 'drifting':
                        node._joint_state.position[2] += .01
            scope = {'copy': copy, 'time': NS(monotonic=lambda: clock.value, sleep=advance),
                     'rclpy': NS(ok=lambda: True)}
            exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), scope)
            result = scope['wait_for_stable_arm'](node)
            self.assertEqual(result is not None, mode == 'stable')


if __name__ == '__main__':
    unittest.main()

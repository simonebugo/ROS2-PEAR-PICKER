"""Offline tests of the actual trigger guard; no ROS installation required."""
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import time
import unittest


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


if __name__ == '__main__':
    unittest.main()

"""Geometry regression tests; no ROS, numpy or OpenCV required."""
import math
from pathlib import Path
import sys
import unittest

root = Path(__file__).resolve().parents[1]
for folder in (root / 'rbt08pkg/scripts', root / 'src/rbt08pkg/scripts'):
    if (folder / 'pear_center.py').is_file():
        sys.path.insert(0, str(folder))
        break
from pear_center import estimate_center


class CenterTests(unittest.TestCase):
    def test_central_ray_sphere(self):
        self.assertEqual(estimate_center((0., 0., 1.), (0., 0., 0.), .075), (0., 0., 1.075))

    def test_shifted_camera_oblique_ray(self):
        origin = (10., -2., 1.)
        direction = (0.6, 0., 0.8)
        surface = tuple(o + 2 * d for o, d in zip(origin, direction))
        center = estimate_center(surface, origin, .075)
        for actual, o, d in zip(center, origin, direction):
            self.assertAlmostEqual(actual, o + 2.075 * d)
        self.assertAlmostEqual(math.dist(center, surface), .075)

    def test_logged_measurement(self):
        center = estimate_center((.816192797451448, .4881203882299332, 1.3317399439919748),
                                 (.15, -.55, 1.75), .075)
        self.assertLess(math.dist(center, (.85, .55, 1.30)), .015)

    def test_reject_invalid_geometry(self):
        for radius in (0., -1., float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                estimate_center((0., 0., 1.), (0., 0., 0.), radius)
        with self.assertRaises(ValueError):
            estimate_center((1., 2., 3.), (1., 2., 3.), .075)
        with self.assertRaises(ValueError):
            estimate_center((float('nan'), 0., 1.), (0., 0., 0.), .075)


if __name__ == '__main__':
    unittest.main()

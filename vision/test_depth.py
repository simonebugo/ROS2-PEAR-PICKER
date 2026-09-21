"""Offline tests: no ROS, Gazebo or robot required."""
from dataclasses import replace
import unittest

import cv2
import numpy as np
from segmentation_paths import add_segmentation_path

add_segmentation_path()
from pear_depth import DepthConfig, Intrinsics, InvalidDepth, estimate_surface


class SurfaceTests(unittest.TestCase):
    def setUp(self):
        self.camera = Intrinsics(100, 80, 100., 200., 50., 40.)
        self.mask = np.zeros((80, 100), np.uint8)
        cv2.circle(self.mask, (65, 30), 15, 255, -1)
        self.depth = np.full((80, 100), 1.2, np.float32)

    def estimate(self, **kwargs):
        return estimate_surface(kwargs.get('mask', self.mask), kwargs.get('depth', self.depth),
                                kwargs.get('camera', self.camera), kwargs.get('config', DepthConfig()))

    def test_off_axis_plane(self):
        result = self.estimate()
        self.assertEqual(result['pixel'], (65, 30))
        np.testing.assert_allclose(result['point'], (.18, -.06, 1.2), atol=1e-7)

    def test_background_ignored(self):
        self.depth[self.mask == 0] = 4.8
        self.assertAlmostEqual(self.estimate()['point'][2], 1.2, places=6)

    def test_empty_target(self):
        with self.assertRaisesRegex(InvalidDepth, 'NO_TARGET'):
            self.estimate(mask=np.zeros_like(self.mask))

    def test_tiny_target(self):
        mask = np.zeros_like(self.mask)
        mask[30, 65] = 255
        with self.assertRaises(InvalidDepth):
            self.estimate(mask=mask)

    def test_invalid_depths(self):
        for value in (np.nan, np.inf, 0., -1., 6.):
            with self.subTest(value=value), self.assertRaises(InvalidDepth):
                self.estimate(depth=np.full_like(self.depth, value))

    def test_invalid_centre_uses_real_ray(self):
        self.depth[30, 65] = np.nan
        result = self.estimate()
        self.assertNotEqual(result['pixel'], (65, 30))
        u, v = result['pixel']
        z = float(self.depth[v, u])
        self.assertEqual(result['point'], ((u-50)*z/100, (v-40)*z/200, z))

    def test_sparse_depth_rejected(self):
        self.depth[:] = np.nan
        self.depth[30, 65] = 1.2
        with self.assertRaises(InvalidDepth):
            self.estimate()

    def test_outlier_robustness(self):
        self.depth[30, 65] = 4.0
        self.assertAlmostEqual(self.estimate()['point'][2], 1.2, places=6)

    def test_depth_discontinuity_rejected(self):
        self.depth[:, :65] = 2.0
        with self.assertRaisesRegex(InvalidDepth, 'incoerente'):
            self.estimate()

    def test_shapes_and_types(self):
        for kwargs in ({'mask': self.mask.astype(float)}, {'depth': self.depth.astype(np.uint16)},
                       {'depth': self.depth[:20]}, {'mask': self.mask[:, :20]}):
            with self.subTest(kwargs=list(kwargs)), self.assertRaises(InvalidDepth):
                self.estimate(**kwargs)

    def test_calibration_rejected(self):
        for camera in (replace(self.camera, fx=0), replace(self.camera, fy=np.nan)):
            with self.assertRaises(ValueError):
                self.estimate(camera=camera)

    def test_noise_stability(self):
        rng = np.random.default_rng(42)
        z_values = []
        for _ in range(20):
            noisy = self.depth + rng.normal(0, .002, self.depth.shape).astype(np.float32)
            z_values.append(self.estimate(depth=noisy)['point'][2])
        self.assertLess(np.ptp(z_values), .003)

    def test_parameters(self):
        for config in (DepthConfig(erosion_size=2), DepthConfig(min_valid_fraction=0),
                       DepthConfig(max_depth_m=.05), DepthConfig(max_depth_spread_m=np.inf)):
            with self.assertRaises(ValueError):
                self.estimate(config=config)


if __name__ == '__main__':
    unittest.main()

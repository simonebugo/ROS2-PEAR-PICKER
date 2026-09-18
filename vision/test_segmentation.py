"""Run without Gazebo: python3 vision/test_segmentation.py -v."""

from dataclasses import replace
import unittest

import cv2
import numpy as np

from segmentation_paths import add_segmentation_path

add_segmentation_path()
from pear_segmentation import SegmentationConfig, annotate, segment_pear


def sample_image(color=(13, 209, 255)):
    image = np.full((240, 320, 3), (46, 107, 61), dtype=np.uint8)
    cv2.ellipse(image, (160, 130), (22, 29), 0, 0, 360, color, -1)
    cv2.circle(image, (160, 105), 13, color, -1)
    return image


class SegmentationTests(unittest.TestCase):
    def test_yellow_pear_geometry(self):
        result = segment_pear(sample_image())
        self.assertIsNotNone(result.candidate)
        self.assertEqual(result.valid_candidates, 1)
        self.assertAlmostEqual(result.candidate.bbox_center[0], 160, delta=1)
        self.assertGreater(result.candidate.height, result.candidate.width)
        self.assertGreater(result.candidate.area, 1000)
        self.assertGreater(result.candidate.fill_ratio, 0.4)

    def test_world_body_and_ambient_colors(self):
        for color in ((13, 209, 255), (8, 173, 230), (4, 70, 90)):
            with self.subTest(color=color):
                self.assertIsNotNone(segment_pear(sample_image(color)).candidate)

    def test_scene_without_yellow_has_no_target(self):
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        for i, color in enumerate(((46, 107, 61), (18, 51, 97), (128, 128, 128), (245, 209, 173))):
            cv2.rectangle(image, (i * 80, 0), (i * 80 + 79, 239), color, -1)
        result = segment_pear(image)
        self.assertIsNone(result.candidate)
        self.assertEqual(np.count_nonzero(result.target_mask), 0)

    def test_reject_non_yellow_fruits(self):
        for color in ((0, 0, 255), (255, 0, 0), (0, 255, 0), (160, 160, 160)):
            with self.subTest(color=color):
                self.assertIsNone(segment_pear(sample_image(color)).candidate)

    def test_too_dark_or_desaturated(self):
        for color in ((1, 15, 20), (220, 240, 245)):
            with self.subTest(color=color):
                self.assertIsNone(segment_pear(sample_image(color)).candidate)

    def test_noise_removed(self):
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        image[::13, ::13] = (0, 220, 255)
        result = segment_pear(image)
        self.assertIsNone(result.candidate)
        self.assertEqual(np.count_nonzero(result.mask), 0)

    def test_small_hole_closed(self):
        image = sample_image()
        image[129:132, 159:162] = 0
        self.assertEqual(segment_pear(image).target_mask[130, 160], 255)

    def test_largest_valid_component_wins(self):
        image = sample_image()
        cv2.circle(image, (50, 70), 12, (0, 220, 255), -1)
        result = segment_pear(image)
        self.assertEqual(result.valid_candidates, 2)
        self.assertEqual(result.mask[70, 50], 255)
        self.assertEqual(result.target_mask[70, 50], 0)
        self.assertEqual(result.target_mask[130, 160], 255)

    def test_reject_huge_region(self):
        image = np.full((240, 320, 3), (0, 220, 255), dtype=np.uint8)
        result = segment_pear(image)
        self.assertIsNone(result.candidate)
        self.assertGreater(np.count_nonzero(result.mask), 0)

    def test_reject_thin_region(self):
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.rectangle(image, (20, 80), (280, 90), (0, 220, 255), -1)
        self.assertIsNone(segment_pear(image).candidate)

    def test_reject_sparse_ring(self):
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2.circle(image, (160, 120), 40, (0, 220, 255), 3)
        self.assertIsNone(segment_pear(image).candidate)

    def test_masks_and_input_unchanged(self):
        image = sample_image()
        original = image.copy()
        result = segment_pear(image)
        output = annotate(image, result)
        np.testing.assert_array_equal(image, original)
        self.assertEqual(output.shape, image.shape)
        self.assertEqual(result.target_mask.dtype, np.uint8)
        self.assertEqual(result.target_mask.shape, image.shape[:2])
        self.assertTrue(set(np.unique(result.target_mask)) <= {0, 255})
        self.assertTrue(np.all(result.target_mask <= result.mask))

    def test_new_frame_does_not_reuse_old_target(self):
        self.assertIsNotNone(segment_pear(sample_image()).candidate)
        self.assertIsNone(segment_pear(np.zeros((240, 320, 3), dtype=np.uint8)).candidate)

    def test_invalid_configuration_rejected(self):
        for overrides in ({'hsv_lower': (40, 100, 40)}, {'hsv_upper': (180, 255, 255)},
                          {'opening_size': 2}, {'min_area_px': 0},
                          {'min_aspect_ratio': 2.0}, {'max_area_fraction': float('nan')},
                          {'min_fill_ratio': 1.1}):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                replace(SegmentationConfig(), **overrides).validate()

    def test_invalid_images_rejected(self):
        for image in (None, np.zeros((2, 2), dtype=np.uint8),
                      np.zeros((2, 2, 3), dtype=np.float32), np.zeros((0, 0, 3), dtype=np.uint8)):
            with self.subTest(image=image), self.assertRaises(ValueError):
                segment_pear(image)


if __name__ == '__main__':
    unittest.main()

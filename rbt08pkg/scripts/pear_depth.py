"""Pinhole deprojection of an observed surface patch, without ROS or TF."""

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class DepthConfig:
    erosion_size: int = 3
    sample_radius_px: int = 4
    min_valid_pixels: int = 10
    min_valid_fraction: float = 0.5
    min_depth_m: float = 0.1
    max_depth_m: float = 5.0
    max_depth_spread_m: float = 0.06

    def validate(self):
        if type(self.erosion_size) is not int or not 1 <= self.erosion_size <= 31 or self.erosion_size % 2 != 1:
            raise ValueError('erosion_size must be odd and in [1,31].')
        if type(self.sample_radius_px) is not int or not 1 <= self.sample_radius_px <= 30:
            raise ValueError('sample_radius_px must be in [1,30].')
        if type(self.min_valid_pixels) is not int or self.min_valid_pixels < 1:
            raise ValueError('min_valid_pixels must be positive.')
        values = (self.min_valid_fraction, self.min_depth_m, self.max_depth_m, self.max_depth_spread_m)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Nonfinite depth parameters.')
        if not 0 < self.min_valid_fraction <= 1 or not 0 < self.min_depth_m < self.max_depth_m:
            raise ValueError('Invalid depth range or valid-pixel fraction.')
        if self.max_depth_spread_m <= 0:
            raise ValueError('max_depth_spread_m must be positive.')


@dataclass(frozen=True)
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def validate(self):
        if self.width < 1 or self.height < 1:
            raise ValueError('Invalid CameraInfo dimensions.')
        if not all(math.isfinite(v) for v in (self.fx, self.fy, self.cx, self.cy)) or self.fx <= 0 or self.fy <= 0:
            raise ValueError('Invalid camera intrinsics.')


class InvalidDepth(ValueError):
    pass


def estimate_surface(mask, depth, camera, config=DepthConfig()):
    config.validate()
    camera.validate()
    expected = (camera.height, camera.width)
    if mask.dtype != np.uint8 or mask.ndim != 2 or mask.shape != expected:
        raise InvalidDepth('mono8 mask does not match CameraInfo.')
    if depth.ndim != 2 or depth.shape != expected or not np.issubdtype(depth.dtype, np.floating):
        raise InvalidDepth('Metric depth does not match CameraInfo.')
    if not np.any(mask):
        raise InvalidDepth('NO_TARGET')
    interior = (mask > 0).astype(np.uint8)
    if config.erosion_size > 1:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (config.erosion_size, config.erosion_size))
        interior = cv2.erode(interior, kernel, borderType=cv2.BORDER_CONSTANT, borderValue=0)
    if not np.any(interior):
        raise InvalidDepth('Mask too small after erosion.')
    # Select a patch far from the silhouette, usually inside the pear body.
    distances = cv2.distanceTransform(interior, cv2.DIST_L2, 5)
    center_v, center_u = np.unravel_index(int(distances.argmax()), distances.shape)
    radius = config.sample_radius_px
    y0, y1 = max(0, center_v - radius), min(camera.height, center_v + radius + 1)
    x0, x1 = max(0, center_u - radius), min(camera.width, center_u + radius + 1)
    v, u = np.mgrid[y0:y1, x0:x1]
    selected = (interior[y0:y1, x0:x1] > 0) & ((u - center_u) ** 2 + (v - center_v) ** 2 <= radius ** 2)
    samples = depth[y0:y1, x0:x1]
    valid = selected & np.isfinite(samples) & (samples >= config.min_depth_m) & (samples <= config.max_depth_m)
    count, total = int(valid.sum()), int(selected.sum())
    if count < config.min_valid_pixels or count / max(1, total) < config.min_valid_fraction:
        raise InvalidDepth(f'Insufficient valid depth: {count}/{total} pixel.')
    values = samples[valid].astype(np.float64)
    median = float(np.median(values))
    spread = float(np.percentile(values, 90) - np.percentile(values, 10))
    if spread > config.max_depth_spread_m:
        raise InvalidDepth(f'Inconsistent depth in the patch: spread={spread:.4f} m.')
    # Use a real valid sample nearest to the median, then closest to patch centre.
    # This avoids pairing an invalid centre pixel with a depth from another ray.
    uv_u, uv_v = u[valid], v[valid]
    index = int(np.lexsort(((uv_u - center_u) ** 2 + (uv_v - center_v) ** 2, np.abs(values - median)))[0])
    pixel_u, pixel_v, z = int(uv_u[index]), int(uv_v[index]), float(values[index])
    point = ((pixel_u - camera.cx) * z / camera.fx,
             (pixel_v - camera.cy) * z / camera.fy, z)
    return {
        'point': point, 'pixel': (pixel_u, pixel_v), 'median_depth_m': median,
        'depth_spread_m': spread, 'valid_pixels': count, 'selected_pixels': total,
    }

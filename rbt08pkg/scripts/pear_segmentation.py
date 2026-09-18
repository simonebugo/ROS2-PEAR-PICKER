"""Pure OpenCV segmentation, independent of ROS and of the robot pose."""

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class SegmentationConfig:
    hsv_lower: tuple = (18, 100, 45)
    hsv_upper: tuple = (38, 255, 255)
    opening_size: int = 3
    closing_size: int = 5
    min_area_px: int = 80
    max_area_fraction: float = 0.15
    min_aspect_ratio: float = 0.35
    max_aspect_ratio: float = 1.8
    min_fill_ratio: float = 0.40

    def validate(self):
        for bounds in (self.hsv_lower, self.hsv_upper):
            if len(bounds) != 3:
                raise ValueError('HSV deve contenere tre interi: H, S, V.')
            for v, limit in zip(bounds, (179, 255, 255)):
                if type(v) is not int or not 0 <= v <= limit:
                    raise ValueError('HSV: H in [0,179], S e V in [0,255].')
        if any(lo > hi for lo, hi in zip(self.hsv_lower, self.hsv_upper)):
            raise ValueError('Ogni soglia HSV inferiore deve essere <= alla superiore.')
        for size in (self.opening_size, self.closing_size):
            if type(size) is not int or not 1 <= size <= 31 or size % 2 != 1:
                raise ValueError('Kernel: interi dispari in [1,31]; 1 disabilita il filtro.')
        if type(self.min_area_px) is not int or self.min_area_px < 1:
            raise ValueError('min_area_px deve essere un intero positivo.')
        values = (self.max_area_fraction, self.min_aspect_ratio,
                  self.max_aspect_ratio, self.min_fill_ratio)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('I parametri geometrici devono essere finiti.')
        if not 0 < self.max_area_fraction <= 1 or not 0 < self.min_fill_ratio <= 1:
            raise ValueError('Le frazioni devono essere in (0,1].')
        if not 0 < self.min_aspect_ratio <= self.max_aspect_ratio:
            raise ValueError('Intervallo del rapporto larghezza/altezza non valido.')


@dataclass(frozen=True)
class Candidate:
    x: int
    y: int
    width: int
    height: int
    area: int
    centroid: tuple
    fill_ratio: float

    @property
    def bbox_center(self):
        # BoundingBox2D uses geometric box centre, not component centroid.
        return (self.x + self.width / 2.0, self.y + self.height / 2.0)


@dataclass
class SegmentationResult:
    mask: np.ndarray
    target_mask: np.ndarray
    candidate: Candidate | None
    valid_candidates: int


def segment_pear(image, config=SegmentationConfig()):
    config.validate()
    if (not isinstance(image, np.ndarray) or image.dtype != np.uint8
            or image.ndim != 3 or image.shape[2] != 3 or image.size == 0):
        raise ValueError('Attesa immagine BGR uint8 non vuota con tre canali.')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(config.hsv_lower, dtype=np.uint8),
                       np.array(config.hsv_upper, dtype=np.uint8))
    for operation, size in ((cv2.MORPH_OPEN, config.opening_size),
                            (cv2.MORPH_CLOSE, config.closing_size)):
        if size > 1:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
            mask = cv2.morphologyEx(mask, operation, kernel)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    target_mask = np.zeros(mask.shape, dtype=np.uint8)
    best = None
    best_label = None
    valid = 0
    for index in range(1, count):
        x, y, width, height, area = map(int, stats[index])
        aspect, fill = width / height, area / (width * height)
        if (area < config.min_area_px or area > mask.size * config.max_area_fraction
                or not config.min_aspect_ratio <= aspect <= config.max_aspect_ratio
                or fill < config.min_fill_ratio):
            continue
        valid += 1
        if best is None or area > best.area:
            best = Candidate(x, y, width, height, area, tuple(centroids[index]), fill)
            best_label = index
    if best_label is not None:
        target_mask[labels == best_label] = 255
    return SegmentationResult(mask, target_mask, best, valid)


def annotate(image, result):
    annotated = image.copy()
    candidate = result.candidate
    label = 'HSV: nessun target'
    if candidate is not None:
        x, y, w, h = candidate.x, candidate.y, candidate.width, candidate.height
        cv2.rectangle(annotated, (x, y), (x + w - 1, y + h - 1), (0, 255, 0), 2)
        cv2.circle(annotated, tuple(round(v) for v in candidate.bbox_center), 4, (0, 0, 255), -1)
        label = f'pear HSV: area={candidate.area}, fill={candidate.fill_ratio:.2f}'
    cv2.putText(annotated, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (255, 255, 255), 2)
    return annotated

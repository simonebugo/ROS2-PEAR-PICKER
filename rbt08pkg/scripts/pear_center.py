"""Approximate a fruit centre behind its observed surface along the camera ray."""
import math


def estimate_center(surface, camera_origin, radius_m):
    if len(surface) != 3 or len(camera_origin) != 3:
        raise ValueError('Expected three-dimensional coordinates.')
    if not all(math.isfinite(v) for v in (*surface, *camera_origin, radius_m)) or radius_m <= 0:
        raise ValueError('Finite coordinates and a positive radius are required.')
    ray = tuple(p - o for p, o in zip(surface, camera_origin))
    distance = math.sqrt(sum(v * v for v in ray))
    if distance <= 1e-9:
        raise ValueError('Point coincides with the camera origin.')
    return tuple(p + radius_m * v / distance for p, v in zip(surface, ray))

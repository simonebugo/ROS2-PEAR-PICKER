"""Shared robot-side approach geometry, without ROS or NumPy dependencies."""
import math


def quaternion_rotation(q):
    """Return the local-to-parent matrix of a unit (x, y, z, w) quaternion."""
    x, y, z, w = q
    return (
        (1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)),
        (2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)),
        (2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)),
    )


def robot_side_grasp_offset(position, pear_rotation):
    if not all(math.isfinite(v) for v in position):
        raise ValueError('Non-finite robot-to-pear direction.')
    local_x = sum(pear_rotation[i][0] * position[i] for i in range(3))
    local_y = sum(pear_rotation[i][1] * position[i] for i in range(3))
    projection = math.hypot(local_x, local_y)
    if (not math.isfinite(projection) or
            projection <= 1e-9 * max(1., math.sqrt(sum(v*v for v in position)))):
        raise ValueError('Robot-to-pear direction has no usable projection in pear XY.')
    return math.atan2(local_y, local_x)

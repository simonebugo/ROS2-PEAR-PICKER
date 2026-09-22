"""Checks on a Cartesian result before it is sent to an action server."""
import math


def cartesian_problem(result, initial_state, max_joint_step=.35):
    if result.error_code.val != 1:
        return f'Cartesian planning failed: code {result.error_code.val}.'
    if not math.isfinite(result.fraction) or result.fraction < 1.-1e-6:
        return f'Incomplete Cartesian trajectory ({result.fraction*100:.1f}%): execution skipped.'
    trajectory = result.solution.joint_trajectory
    names = list(trajectory.joint_names)
    initial = dict(zip(initial_state.name, initial_state.position))
    if not trajectory.points or not names or any(name not in initial for name in names):
        return 'Empty Cartesian trajectory or missing initial joints.'
    previous = [initial[name] for name in names]
    for index, point in enumerate(trajectory.points):
        if len(point.positions) != len(names) or not all(math.isfinite(v) for v in point.positions):
            return 'Invalid joint coordinates in the Cartesian trajectory.'
        for name, before, after in zip(names, previous, point.positions):
            if abs(after-before) > max_joint_step:
                return f'IK jump on {name} at point {index}: {abs(after-before):.3f} rad. Execution skipped.'
        previous = point.positions
    return None

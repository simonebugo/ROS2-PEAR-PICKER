"""Checks on a Cartesian result before it is sent to an action server."""
import math


def cartesian_problem(result, initial_state, max_joint_step=.35):
    if result.error_code.val != 1:
        return f'Calcolo cartesiano fallito: codice {result.error_code.val}.'
    if not math.isfinite(result.fraction) or result.fraction < 1.-1e-6:
        return f'Traiettoria cartesiana incompleta ({result.fraction*100:.1f}%): nessuna esecuzione.'
    trajectory = result.solution.joint_trajectory
    names = list(trajectory.joint_names)
    initial = dict(zip(initial_state.name, initial_state.position))
    if not trajectory.points or not names or any(name not in initial for name in names):
        return 'Traiettoria cartesiana vuota o giunti iniziali mancanti.'
    previous = [initial[name] for name in names]
    for index, point in enumerate(trajectory.points):
        if len(point.positions) != len(names) or not all(math.isfinite(v) for v in point.positions):
            return 'Coordinate articolari non valide nella traiettoria cartesiana.'
        for name, before, after in zip(names, previous, point.positions):
            if abs(after-before) > max_joint_step:
                return f'Salto IK su {name} al punto {index}: {abs(after-before):.3f} rad. Nessuna esecuzione.'
        previous = point.positions
    return None

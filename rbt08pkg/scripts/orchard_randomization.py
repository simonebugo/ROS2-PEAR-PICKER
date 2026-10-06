"""Seeded startup geometry and numerical reachability screening; no ROS targets.

The manifest feeds collision geometry and the known branch-based grasp yaw.
The robot's target position still comes exclusively from vision.
Numerical IK checks joint limits, not mesh collisions or dynamic execution.
"""
import hashlib
import json
import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET

import numpy as np


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def transform(xyz=(0., 0., 0.), rpy=(0., 0., 0.)):
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = rotation(rpy), xyz
    return t


def numbers(text):
    return [float(v) for v in text.split()]


def origin(joint):
    element = joint.find('origin')
    return (np.eye(4) if element is None else transform(
        numbers(element.get('xyz', '0 0 0')), numbers(element.get('rpy', '0 0 0'))))


def rotation_error(target, actual):
    delta = target @ actual.T
    vector = np.array([delta[2, 1]-delta[1, 2], delta[0, 2]-delta[2, 0], delta[1, 0]-delta[0, 1]]) / 2
    sine = np.linalg.norm(vector)
    cosine = np.clip((np.trace(delta)-1)/2, -1., 1.)
    if sine > 1e-8:
        return vector * math.atan2(sine, cosine) / sine
    if cosine > 0:
        return vector
    # Rotation by pi: the skew part vanishes, but the axis still exists.
    _, axes = np.linalg.eigh((delta + delta.T) / 2)
    return math.pi * axes[:, -1]


class ArmKinematics:
    """Read the project's numeric URDF joints; fail on unsupported joint types."""
    def __init__(self, share):
        joints = []
        for filename in ('rbt10.urdf', 'gripper.urdf.xacro'):
            joints.extend(ET.parse(Path(share) / 'urdf' / filename).getroot().findall('joint'))
        self.by_child = {j.find('child').get('link'): j for j in joints}
        self.chain = self.path('grasp_frame')
        moving = [j for j in self.chain if j.get('type') != 'fixed']
        if [j.get('name') for j in moving] != ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']:
            raise ValueError('The world -> grasp_frame chain must contain j1..j6.')
        if any(j.get('type') != 'revolute' for j in moving):
            raise ValueError('The IK filter requires bounded revolute joints.')
        self.names = [j.get('name') for j in moving]
        self.lower = np.array([float(j.find('limit').get('lower')) for j in moving])
        self.upper = np.array([float(j.find('limit').get('upper')) for j in moving])
        self.steps = []
        for joint in self.chain:
            axis = None if joint.get('type') == 'fixed' else np.array(numbers(joint.find('axis').get('xyz')))
            if axis is not None:
                axis /= np.linalg.norm(axis)
            self.steps.append((origin(joint), axis))
        self.base = self.fixed_transform('base_link')

    def path(self, tip):
        chain, seen = [], set()
        while tip != 'world':
            if tip in seen or tip not in self.by_child:
                raise ValueError(f'Incomplete URDF chain for {tip}.')
            seen.add(tip)
            joint = self.by_child[tip]
            chain.append(joint)
            tip = joint.find('parent').get('link')
        return list(reversed(chain))

    def fixed_transform(self, tip):
        t = np.eye(4)
        for j in self.path(tip):
            if j.get('type') != 'fixed':
                raise ValueError(f'{tip} is not fixed relative to world.')
            t = t @ origin(j)
        return t

    def forward(self, q):
        t, axes, points, i = np.eye(4), [], [], 0
        for offset, axis in self.steps:
            t = t @ offset
            if axis is not None:
                points.append(t[:3, 3].copy())
                axes.append(t[:3, :3] @ axis)
                x, y, z = axis
                skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
                r = np.eye(4)
                r[:3, :3] += math.sin(q[i])*skew + (1-math.cos(q[i]))*(skew @ skew)
                t = t @ r
                i += 1
        jacobian = np.array([np.r_[np.cross(a, t[:3, 3]-p), a] for a, p in zip(axes, points)]).T
        return t, jacobian

    def solve(self, target, previous=None):
        # A separate, fixed RNG keeps geometry independent of IK iteration counts.
        seeds = [np.zeros(6)] + list(np.random.default_rng(1234).uniform(self.lower, self.upper, (7, 6)))
        if previous is not None:
            seeds.insert(0, np.array(previous))
        for seed in seeds:
            q = seed.copy()
            for _ in range(180):
                t, jac = self.forward(q)
                error = np.r_[target[:3, 3]-t[:3, 3], rotation_error(target[:3, :3], t[:3, :3])]
                if np.linalg.norm(error[:3]) < .0005 and np.linalg.norm(error[3:]) < .003:
                    return q.tolist()
                step = jac.T @ np.linalg.solve(jac @ jac.T + .015**2*np.eye(6), error)
                step *= min(1., .25/max(np.linalg.norm(step), 1e-12))
                q = np.clip(q + step, self.lower, self.upper)
        return None

    def targets(self, pear, approach_yaw_world=None):
        p = (np.linalg.inv(self.base) @ np.r_[pear, 1.])[:3]
        yaw = math.atan2(p[1], p[0])
        attitude = rotation((0., math.pi/2, yaw))
        if approach_yaw_world is not None:
            attitude = self.base[:3, :3].T @ rotation((0., math.pi/2, approach_yaw_world))
        direction = attitude[:, 2]
        pre = p - .25*direction
        # With the rotated wrist, the old 20 cm lift puts a finger into the
        # fruit branch. Its 10 cm intermediate waypoint is collision-free in
        # the live seed-42 scene; keep the first lift within that clearance.
        lift_height = .10 if approach_yaw_world is not None else .20
        targets = {}
        for name, xyz in (
            ('pre_grasp', pre), ('approach_mid', p-.125*direction), ('grasp', p),
            ('lift_mid', pre + [0., 0., lift_height/2]),
            ('lift', pre + [0., 0., lift_height])):
            target = transform(xyz)
            target[:3, :3] = attitude
            targets[name] = self.base @ target
        return targets


def reference_approach_yaw(share):
    """World yaw of the successful radial approach in the original world."""
    base = ArmKinematics(share).base
    root = ET.parse(Path(share)/'worlds/orchard.world.sdf')
    pear = numbers(root.findtext(".//model[@name='pear']/pose"))[:3]
    return math.atan2(pear[1]-base[1, 3], pear[0]-base[0, 3])


def load_bounds(path):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    expected = {'branch_height_m', 'branch_yaw_deg', 'branch_elevation_deg', 'pear_distance_m',
                'max_attempts', 'image_margin_px', 'min_body_diameter_px'}
    if set(data) != expected:
        raise ValueError(f'Expected bounds keys: {sorted(expected)}')
    for name in ('branch_height_m', 'branch_yaw_deg', 'branch_elevation_deg', 'pear_distance_m'):
        bounds = data[name]
        if (not isinstance(bounds, list) or len(bounds) != 2 or
                not all(isinstance(v, (float, int)) and math.isfinite(v) for v in bounds) or bounds[0] > bounds[1]):
            raise ValueError(f'Invalid bounds: {name}')
    if type(data['max_attempts']) is not int or not 1 <= data['max_attempts'] <= 1000:
        raise ValueError('max_attempts must be between 1 and 1000.')
    if not (0 < data['branch_height_m'][0] and 0 < data['pear_distance_m'][0]):
        raise ValueError('Height and distance must be positive.')
    if not (-80 <= data['branch_yaw_deg'][0] <= data['branch_yaw_deg'][1] <= 80 and
            -30 <= data['branch_elevation_deg'][0] <= data['branch_elevation_deg'][1] <= 30):
        raise ValueError('Angles outside the supported front hemisphere (yaw +/-80 deg, elevation +/-30 deg).')
    for key in ('image_margin_px', 'min_body_diameter_px'):
        if not isinstance(data[key], (float, int)) or not math.isfinite(data[key]) or data[key] < 0:
            raise ValueError(f'{key} is invalid.')
    return data


def camera_accepts(share, arm, pear, radius, bounds):
    cam = ET.parse(Path(share)/'urdf/camera.gazebo').find('.//camera')
    width, height = int(cam.findtext('image/width')), int(cam.findtext('image/height'))
    f = width / (2*math.tan(float(cam.findtext('horizontal_fov'))/2))
    near, far = float(cam.findtext('clip/near')), float(cam.findtext('clip/far'))
    inverse = np.linalg.inv(arm.fixed_transform('table_camera_link_optical'))
    # Bound the body, neck and visible stem with spheres, including image margin.
    for offset, r in ((0., radius), (.065, .052), (.14, .041)):
        x, y, z = (inverse @ np.r_[np.array(pear)+[0., 0., offset], 1.])[:3]
        if not near < z-r or z+r >= far:
            return False
        margin = bounds['image_margin_px']
        if ((abs(x)+r)/(z-r)*f >= width/2-margin or
                (abs(y)+r)/(z-r)*f >= height/2-margin):
            return False
        if offset == 0. and 2*f*radius/z < bounds['min_body_diameter_px']:
            return False
    return True


def generate_scenario(share, bounds_file, seed, grasp_orientation='branch'):
    if grasp_orientation not in ('branch', 'radial'):
        raise ValueError('grasp_orientation must be branch or radial.')
    share = Path(share)
    config = load_bounds(bounds_file)
    sdf = ET.parse(share/'worlds/orchard.world.sdf')
    tree = sdf.find(".//model[@name='pear_tree']")
    tree_pose = numbers(tree.findtext('pose'))
    if any(abs(v) > 1e-9 for v in tree_pose[3:]):
        raise ValueError('The generator requires an upright, unrotated trunk.')
    branch = tree.find("link/collision[@name='fruit_branch_collision']")
    length = float(branch.findtext('geometry/cylinder/length'))
    branch_radius = float(branch.findtext('geometry/cylinder/radius'))
    pear_radius = float(sdf.findtext(".//model[@name='pear']/link/collision[@name='pear_body_collision']/geometry/sphere/radius"))
    if config['pear_distance_m'][1] > length-.02:
        raise ValueError('pear_distance_m exceeds the usable branch length.')
    arm, rng, rejected = ArmKinematics(share), random.Random(seed), {}
    nominal_yaw = reference_approach_yaw(share)
    for attempt in range(1, config['max_attempts']+1):
        sampled = {name: rng.uniform(*config[name]) for name in (
            'branch_height_m', 'branch_yaw_deg', 'branch_elevation_deg', 'pear_distance_m')}
        yaw = math.pi + math.radians(sampled['branch_yaw_deg'])
        elevation = math.radians(sampled['branch_elevation_deg'])
        direction = np.array([math.cos(elevation)*math.cos(yaw), math.cos(elevation)*math.sin(yaw), math.sin(elevation)])
        pivot = np.array([tree_pose[0], tree_pose[1], sampled['branch_height_m']])
        center = pivot + length/2*direction
        # Keep the pear vertical, with its stem tip at the selected branch point.
        pear = pivot + sampled['pear_distance_m']*direction - [0., 0., .18]
        if (pear[0] >= tree_pose[0]-.2 or pear[2] <= .2 or
                not camera_accepts(share, arm, pear, pear_radius, config)):
            rejected['geometry_or_camera'] = rejected.get('geometry_or_camera', 0)+1
            continue
        approach_yaw = (nominal_yaw + math.radians(sampled['branch_yaw_deg'])
                        if grasp_orientation == 'branch' else None)
        solutions, previous = {}, None
        for name, target in arm.targets(pear, approach_yaw).items():
            q = arm.solve(target, previous)
            if q is None:
                rejected['ik_'+name] = rejected.get('ik_'+name, 0)+1
                break
            solutions[name], previous = q, q
        else:
            inputs = [share/'worlds/orchard.world.sdf', share/'urdf/rbt10.urdf',
                      share/'urdf/gripper.urdf.xacro', share/'urdf/camera.gazebo']
            return {
                'version': 1, 'seed': seed, 'attempt': attempt, 'bounds': config,
                'sampled': sampled, 'rejected': rejected,
                'branch_pose_world': center.tolist()+[0., math.pi/2-elevation, yaw],
                'branch_pose_tree': (center-np.array(tree_pose[:3])).tolist()+[0., math.pi/2-elevation, yaw],
                'branch_length': length, 'branch_radius': branch_radius,
                # The fruit stays vertical; its local frame follows the
                # branch's horizontal yaw relative to the nominal world.
                'pear_pose_world': pear.tolist()+[0., 0., math.radians(sampled['branch_yaw_deg'])],
                'grasp_orientation': grasp_orientation,
                'grasp_approach_yaw_world': approach_yaw,
                'ik_joint_names': arm.names, 'ik_solutions': solutions,
                'screening': 'numeric joint-limit IK and camera frustum; not collision-free path certification',
                'input_sha256': {str(p.relative_to(share)): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
            }
    raise RuntimeError(f'No valid scenario after {config["max_attempts"]} attempts; seed={seed}, rejected={rejected}.')


def load_scenario(path):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if data.get('version') != 1:
        raise ValueError('Unsupported scenario version.')
    mode = data.get('grasp_orientation', 'radial')  # Older manifests used radial targets.
    if mode not in ('branch', 'radial'):
        raise ValueError('Invalid grasp_orientation in scenario.')
    angle = data.get('grasp_approach_yaw_world')
    if mode == 'branch' and (not isinstance(angle, (float, int)) or not math.isfinite(angle)):
        raise ValueError('Missing or invalid branch-based grasp yaw.')
    if mode == 'radial' and angle is not None:
        raise ValueError('Radial scenario must not specify a fixed approach yaw.')
    for key in ('branch_pose_world', 'branch_pose_tree', 'pear_pose_world'):
        if len(data[key]) != 6 or not all(math.isfinite(v) for v in data[key]):
            raise ValueError(f'Invalid scenario pose: {key}')
    for key in ('branch_length', 'branch_radius'):
        if not math.isfinite(data[key]) or data[key] <= 0:
            raise ValueError(f'Invalid scenario dimension: {key}')
    return data


def write_scenario(share, scenario, directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    sdf = ET.parse(Path(share)/'worlds/orchard.world.sdf')
    text = lambda values: ' '.join(format(v, '.17g') for v in values)
    sdf.find(".//model[@name='pear']/pose").text = text(scenario['pear_pose_world'])
    for kind in ('collision', 'visual'):
        suffix = 'collision' if kind == 'collision' else 'visual'
        sdf.find(f".//model[@name='pear_tree']/link/{kind}[@name='fruit_branch_{suffix}']/pose").text = text(scenario['branch_pose_tree'])
    world_path, manifest = directory/'orchard.world.sdf', directory/'scenario.json'
    sdf.write(world_path, encoding='utf-8', xml_declaration=True)
    scenario = dict(scenario, generated_world_sha256=hashlib.sha256(world_path.read_bytes()).hexdigest())
    manifest.write_text(json.dumps(scenario, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    return str(world_path), str(manifest)

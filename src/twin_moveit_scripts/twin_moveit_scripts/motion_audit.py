"""Optional motion experiments and evidence. No ROS imports or execution authority."""
import hashlib
import json
import math
import os
from pathlib import Path
import time
import uuid
import xml.etree.ElementTree as ET

DEFAULT_PLANNER = 'RRTConnectkConfigDefault'
ARM_JOINTS = [f'panda_joint{i}' for i in range(1, 8)]
NOT_CHECKED = ['continuous_interpolation_limits', 'continuous_collision',
               'torque', 'jerk_limits', 'grasp_retention', 'hardware_safety',
               'state_freshness', 'task_constraints']


def request_options(configs):
    """Reject unknown planner names rather than letting OMPL silently substitute."""
    planner = os.environ.get('TWIN_PLANNER_ID', DEFAULT_PLANNER)
    if planner not in configs:
        raise ValueError(f'Unknown TWIN_PLANNER_ID: {planner}')
    budget = float(os.environ.get('TWIN_PLANNING_TIME', '5.0'))
    attempts = int(os.environ.get('TWIN_PLANNING_ATTEMPTS', '5'))
    if not math.isfinite(budget) or not 0 < budget <= 30 or not 1 <= attempts <= 10:
        raise ValueError('Invalid planning budget/attempts')
    return dict(planner_id=planner, planning_time=budget, planning_attempts=attempts)


def limit_profile(urdf, overrides, velocity_scale, acceleration_scale):
    """Use URDF positions and configured MoveIt velocity/acceleration overrides."""
    if any(not _finite(s) or not 0 < s <= 1 for s in (velocity_scale, acceleration_scale)):
        raise ValueError('Scaling must be finite and in (0, 1]')
    joints = {j.attrib['name']: j for j in ET.fromstring(urdf).findall('joint')}
    limits = {}
    for name in ARM_JOINTS:
        tag = joints[name].find('limit')
        safety = joints[name].find('safety_controller')
        cfg = overrides.get(name, {})
        lower, upper = float(tag.attrib['lower']), float(tag.attrib['upper'])
        if safety is not None:
            lower = max(lower, float(safety.attrib.get('soft_lower_limit', lower)))
            upper = min(upper, float(safety.attrib.get('soft_upper_limit', upper)))
        limits[name] = dict(position_min=max(lower, float(cfg.get('min_position', lower))),
                            position_max=min(upper, float(cfg.get('max_position', upper))),
                            velocity_max=min(float(tag.attrib['velocity']),
                                float(cfg.get('max_velocity', tag.attrib['velocity']))) * velocity_scale,
                            acceleration_max=float(cfg['max_acceleration']) * acceleration_scale)
        if any(not cfg.get(key, True) for key in ('has_position_limits',
                'has_velocity_limits', 'has_acceleration_limits')):
            raise ValueError(f'{name}: unbounded profile unsupported')
    return limits


def trajectory_data(message):
    traj = message.joint_trajectory
    return dict(joint_names=list(traj.joint_names), points=[dict(
        time=p.time_from_start.sec + p.time_from_start.nanosec * 1e-9,
        positions=list(p.positions), velocities=list(p.velocities),
        accelerations=list(p.accelerations)) for p in traj.points])


def validate(data, limits, expected_names=ARM_JOINTS, stationary_start=None):
    """Structural and sampled bounds only; errors are machine-readable codes."""
    errors = []
    if not isinstance(data, dict) or not isinstance(limits, dict) or not isinstance(data.get('points'), list):
        return dict(sampled_checks_passed=False, errors=['MALFORMED_INPUT'],
                    not_checked=NOT_CHECKED, execution_authorized=False)
    names, points = data.get('joint_names'), data.get('points', [])
    if (not isinstance(names, list) or not names or names != list(expected_names)
            or len(set(names)) != len(names)):
        errors.append('JOINT_ORDER_MISMATCH')
    if not points:
        errors.append('INSUFFICIENT_POINTS')
    if len(points) == 1:
        p = points[0]
        q = p.get('positions', []) if isinstance(p, dict) else None
        valid_hold = (isinstance(q, list) and stationary_start is not None and len(q) == len(expected_names)
                      and len(stationary_start) == len(expected_names)
                      and all(_finite(x) and _finite(y) and abs(x-y) <= 1e-4
                              for x, y in zip(q, stationary_start))
                      and p.get('time') == 0
                      and all(isinstance(p.get(k), list) and len(p[k]) == len(expected_names)
                              and all(_finite(v) and abs(v) <= 1e-9 for v in p[k])
                              for k in ('velocities', 'accelerations')))
        if not valid_hold:
            errors.append('INVALID_STATIONARY_HOLD')
    valid_limits = True
    for name in expected_names:
        lim = limits.get(name, {})
        keys = ('position_min', 'position_max', 'velocity_max', 'acceleration_max')
        if not isinstance(lim, dict) or any(not _finite(lim.get(k)) for k in keys) or (
                lim.get('position_min', 0) >= lim.get('position_max', 0) or
                lim.get('velocity_max', 0) <= 0 or lim.get('acceleration_max', 0) <= 0):
            errors.append(f'INVALID_LIMIT:{name}')
            valid_limits = False
    previous = -1.0
    for i, p in enumerate(points):
        if not isinstance(p, dict):
            errors.append(f'POINT_TYPE:{i}')
            continue
        stamp = p.get('time')
        if not _finite(stamp) or stamp < 0 or stamp <= previous:
            errors.append(f'INVALID_TIME:{i}')
        else:
            previous = stamp
        for field, bound in (('positions', None), ('velocities', 'velocity_max'),
                             ('accelerations', 'acceleration_max')):
            values = p.get(field)
            if not isinstance(values, list) or len(values) != len(expected_names):
                errors.append(f'ARRAY_SHAPE:{i}:{field}')
                continue
            for name, value in zip(expected_names, values):
                if not _finite(value):
                    errors.append(f'NONFINITE:{i}:{name}:{field}')
                elif valid_limits:
                    lim = limits[name]
                    if bound and abs(value) > lim[bound] + 1e-6:
                        errors.append(f'LIMIT:{i}:{name}:{field}')
                    if not bound and not lim['position_min'] - 1e-6 <= value <= lim['position_max'] + 1e-6:
                        errors.append(f'LIMIT:{i}:{name}:{field}')
    return dict(sampled_checks_passed=not errors, errors=errors, not_checked=NOT_CHECKED,
                execution_authorized=False)


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def metrics(data, ee_points=None):
    pts = data['points']
    if not pts:
        return dict(waypoints=0)
    length = sum(math.dist(a['positions'], b['positions']) for a, b in zip(pts, pts[1:]))
    stops = sum(bool(p['velocities']) and max(map(abs, p['velocities'])) < 1e-4 for p in pts[1:-1])
    jerk = []
    for a, b in zip(pts, pts[1:]):
        dt = b['time'] - a['time']
        if dt > 0 and len(a['accelerations']) == len(b['accelerations']) and a['accelerations']:
            jerk.append(math.dist(a['accelerations'], b['accelerations']) / dt)
    return dict(waypoints=len(pts), duration_sim=pts[-1]['time'] - pts[0]['time'],
                joint_path_length_rad=length, interior_stop_samples=stops,
                ee_path_length_m=None if ee_points is None else sum(
                    math.dist(a, b) for a, b in zip(ee_points, ee_points[1:])),
                sampled_acceleration_change_max=max(jerk, default=None))


def interpolated_positions(data, max_dt=.02, max_joint_step=.03, max_samples=20000):
    """Sample JTC q/v/a quintic spans; not a continuous collision certificate."""
    count = 0
    if len(data['points']) == 1:
        yield data['points'][0]['positions']
        return
    for a, b in zip(data['points'], data['points'][1:]):
        dt = b['time'] - a['time']
        divisions = max(1, math.ceil(dt / max_dt), math.ceil(max(
            abs(x-y) for x, y in zip(a['positions'], b['positions'])) / max_joint_step))
        count += divisions + 1
        if count > max_samples:
            raise ValueError('COLLISION_SAMPLE_BUDGET')
        coefficients = []
        for q0, q1, v0, v1, a0, a1 in zip(a['positions'], b['positions'],
                a['velocities'], b['velocities'], a['accelerations'], b['accelerations']):
            delta = q1-q0
            v0, v1, a0, a1 = v0*dt, v1*dt, a0*dt*dt, a1*dt*dt
            coefficients.append((q0, v0, a0/2,
                10*delta-6*v0-4*v1-1.5*a0+.5*a1,
                -15*delta+8*v0+7*v1+1.5*a0-a1,
                6*delta-3*v0-3*v1-.5*a0+.5*a1))
        for i in range(divisions+1):
            u = i/divisions
            yield [sum(c*u**k for k, c in enumerate(poly)) for poly in coefficients]


def record(event, **fields):
    """One process owns a file. Close each append so forced MoveIt exit loses no records."""
    target = os.environ.get('TWIN_MOTION_METRICS')
    if not target:
        return
    row = dict(schema=1, event=event, monotonic_ns=time.monotonic_ns(),
               clock_domain='host_monotonic', pid=os.getpid(), **fields)
    path = Path(target)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a') as stream:
        stream.write(json.dumps(row, allow_nan=False) + '\n')


def identifier():
    return uuid.uuid4().hex


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()

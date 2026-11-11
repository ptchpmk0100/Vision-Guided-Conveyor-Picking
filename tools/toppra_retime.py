#!/usr/bin/env python3
"""Offline TOPPRA reference on the saved JTC quintic path; never dispatches motion.

Optional dependency: toppra. The path is the existing q/v/a polynomial, avoiding
an unmeasured cubic fit to different geometry. Constant scalar-acceleration output
preserves that path; it is not exported as a controller trajectory. A future export
would require a separate interpolation reconstruction and collision check.
"""
import argparse
import json
import math
from pathlib import Path
import time
import numpy as np


class SavedQuinticPath:
    def __init__(self, trajectory):
        points = trajectory['points']
        self.times = np.array([p['time'] for p in points])
        if len(points) < 2 or not np.all(np.diff(self.times) > 0):
            raise ValueError('Need strictly increasing trajectory times')
        q, v, a = (np.array([p[k] for p in points], dtype=float)
                   for k in ('positions', 'velocities', 'accelerations'))
        if q.shape != v.shape or q.shape != a.shape or not np.isfinite([q,v,a]).all():
            raise ValueError('Invalid saved q/v/a')
        self._dof = q.shape[1]
        dt = np.diff(self.times)[:, None]
        delta = q[1:]-q[:-1]
        v0, v1, a0, a1 = v[:-1]*dt, v[1:]*dt, a[:-1]*dt**2, a[1:]*dt**2
        self.coefficients = np.stack((q[:-1], v0, a0/2,
            10*delta-6*v0-4*v1-1.5*a0+.5*a1,
            -15*delta+8*v0+7*v1+1.5*a0-a1,
            6*delta-3*v0-3*v1-.5*a0+.5*a1), axis=2)

    @property
    def dof(self):
        return self._dof

    @property
    def path_interval(self):
        return self.times[[0,-1]]

    @property
    def waypoints(self):
        return None

    def __call__(self, positions, order=0):
        scalar = np.ndim(positions) == 0
        ss = np.atleast_1d(positions).astype(float)
        if order not in (0,1,2) or not np.isfinite(ss).all() or np.any(ss < self.times[0]-1e-8) or np.any(ss > self.times[-1]+1e-8):
            raise ValueError('Invalid path argument')
        ss = np.clip(ss, self.times[0], self.times[-1])
        indices = np.clip(np.searchsorted(self.times, ss, side='right')-1, 0, len(self.times)-2)
        dt = self.times[indices+1]-self.times[indices]
        u = (ss-self.times[indices])/dt
        c = self.coefficients[indices]
        result = np.zeros((len(ss), self.dof))
        for k in range(order,6):
            result += c[:,:,k]*(math.factorial(k)/math.factorial(k-order))*(u**(k-order))[:,None]
        result /= (dt**order)[:,None]
        return result[0] if scalar else result

    def eval(self, s): return self(s)
    def evald(self, s): return self(s,1)
    def evaldd(self, s): return self(s,2)


def compare(row, step=.01):
    import toppra as ta
    import toppra.algorithm as algo
    import toppra.constraint as constraint
    data, limits = row['trajectory'], row['limits']
    if len(data['points']) < 2:
        return dict(command_id=row['command_id'], status='SKIPPED_STATIONARY')
    path = SavedQuinticPath(data)
    names = data['joint_names']
    velocity = np.array([limits[n]['velocity_max'] for n in names])
    acceleration = np.array([limits[n]['acceleration_max'] for n in names])
    position_min = np.array([limits[n]['position_min'] for n in names])
    position_max = np.array([limits[n]['position_max'] for n in names])
    n = max(10, math.ceil((path.times[-1]-path.times[0])/step))
    grid = np.unique(np.r_[path.times,np.linspace(*path.path_interval,n+1)])
    constraints = [constraint.JointVelocityConstraint(velocity),
                   constraint.JointAccelerationConstraint(np.c_[-acceleration,acceleration])]
    started = time.monotonic()
    instance = algo.TOPPRA(constraints, path, gridpoints=grid,
                          solver_wrapper='seidel', parametrizer='ParametrizeConstAccel')
    output = instance.compute_trajectory(0,0)
    elapsed = time.monotonic()-started
    if output is None:
        return dict(command_id=row['command_id'],status='INFEASIBLE',solve_wall_s=elapsed)
    speeds = instance.problem_data.sd_vec
    knot_times = np.r_[0,np.cumsum(2*np.diff(grid)/(speeds[:-1]+speeds[1:]))]
    times = np.unique(np.clip(np.r_[np.linspace(0,output.duration,4001),knot_times,
                                    knot_times-1e-7,knot_times+1e-7],0,output.duration))
    q, v, a = output(times), output(times,1), output(times,2)
    residuals = dict(velocity_excess=float(np.max(abs(v)-velocity)),
                     acceleration_excess=float(np.max(abs(a)-acceleration)),
                     position_excess=float(max(np.max(q-position_max),np.max(position_min-q))))
    finite = np.isfinite(q).all() and np.isfinite(v).all() and np.isfinite(a).all()
    feasible = bool(finite and all(r <= 1e-5 for r in residuals.values()))
    return dict(command_id=row['command_id'],status='SAMPLED_FEASIBLE' if feasible else 'RESIDUAL_FAILED',
        original_duration=data['points'][-1]['time']-data['points'][0]['time'],
        toppra_duration=float(output.duration), solve_wall_s=elapsed,
        gridpoints=len(grid), validation_samples=len(times), residuals=residuals,
        path='exact saved q/v/a quintic; constant scalar acceleration parameterization',
        execution_authorized=False, not_checked=['controller_export_interpolation',
            'continuous_limits','continuous_collision','torque','jerk','grasp_retention'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('records',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--grid-step',type=float,default=.01)
    args = parser.parse_args()
    if not math.isfinite(args.grid_step) or not .0001 <= args.grid_step <= .1:
        parser.error('--grid-step must be finite and between .0001 and .1')
    results = []
    for row in map(json.loads,args.records.read_text().splitlines()):
        if row.get('event') == 'candidate':
            try:
                results.append(compare(row,args.grid_step))
            except Exception as exc:
                results.append(dict(command_id=row['command_id'],status='ERROR',error=str(exc)))
    args.output.write_text(json.dumps(dict(execution_authorized=False, cases=results),indent=2)+'\n')
    print(json.dumps({status:sum(c['status']==status for c in results)
                      for status in sorted({c['status'] for c in results})}))
    raise SystemExit(0 if results and all(c['status'] in ('SAMPLED_FEASIBLE','SKIPPED_STATIONARY') for c in results) else 1)

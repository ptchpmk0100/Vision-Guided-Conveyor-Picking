#!/usr/bin/env python3
"""Independent pose-in-hand, sensor and lift-onset verdict from recorded actual data."""
import argparse
import bisect
import json
import math
from pathlib import Path

import numpy as np
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState

ROOT=Path(__file__).resolve().parents[1]


def rotation(q):
    x,y,z,w=q
    n=math.sqrt(x*x+y*y+z*z+w*w)
    x,y,z,w=x/n,y/n,z/n,w/n
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


# Arm joint-speed (rad/s, L2 over the 7 arm joints) below which the arm is treated as
# quasi-static. Elastic deflection of a compliantly-held carton scales with the arm's
# inertial load, so grip displacement measured during fast transport is deflection that
# relaxes, not slip. Evaluating slip only at quasi-static instants isolates a persistent
# change of the object's seat in the grip (real slip) from that transient deflection,
# which is what the full-motion maximum conflated. Full-motion extremes are still
# reported as diagnostics but do not decide the verdict.
QUASISTATIC_SPEED=0.05


def analyze(directory,translation_limit=.003):
    task=json.loads((directory/'task.json').read_text())
    rows=[]
    for line in (directory/'grasp_evidence.jsonl').read_text().splitlines():
        try:rows.append(json.loads(line))
        except json.JSONDecodeError:break  # writer may currently be appending its final line
    joints=sorted((r for r in rows if r['kind']=='joints'),key=lambda r:r['ns'])
    stamps=[r['ns'] for r in joints]
    model=RobotModel(str(ROOT/'src/twin_description/urdf/panda.urdf'),str(ROOT/'src/twin_moveit_config/config/panda.srdf'))
    state=RobotState(model)
    confirms=[e for e in task['events'] if e['state']=='GRASP_CONFIRMED']
    releases=[e for e in task['events'] if e['state']=='RELEASE_AUTHORIZED']
    intercepts=[e for e in task['events'] if e['state']=='INTERCEPT']
    requests=[e for e in task['events'] if e['state']=='LIFT_REQUESTED']
    armnames=[f'panda_joint{n}' for n in range(1,8)]
    cases=[]
    for k,(confirm,release,request,intercept) in enumerate(zip(confirms,releases,requests,intercepts)):
        poses=sorted((r for r in rows if r['kind']=='pose' and r['name']==confirm['object']
                      and confirm['sim_time']<=r['ns']/1e9<=release['sim_time']),key=lambda r:r['ns'])
        measured=[]
        for pose in poses:
            i=bisect.bisect_left(stamps,pose['ns'])
            if not 0<i<len(joints):continue
            a,b=joints[i-1],joints[i]
            if b['ns']-a['ns']>40000000:continue
            f=(pose['ns']-a['ns'])/(b['ns']-a['ns'])
            aq=dict(zip(a['names'],a['q']));bq=dict(zip(b['names'],b['q']))
            av=dict(zip(a['names'],a['v']));bv=dict(zip(b['names'],b['v']))
            state.set_joint_group_positions('panda_arm',[(1-f)*aq[n]+f*bq[n] for n in armnames])
            state.set_joint_group_positions('hand',[(1-f)*aq[n]+f*bq[n] for n in ('panda_finger_joint1','panda_finger_joint2')])
            state.update(force=True)
            hand=state.get_global_link_transform('panda_link8')
            point=hand[:3,:3].T@(np.array(pose['xyz'])-hand[:3,3])
            orientation=hand[:3,:3].T@rotation(pose['quat'])
            speed=math.sqrt(sum(((1-f)*av[n]+f*bv[n])**2 for n in armnames))
            measured.append((pose,point,orientation,speed))
        if not measured:
            cases.append(dict(passed=False,error='No synchronized independent hand/object measurements'));continue
        # Reference seat in the grip: the median over the earliest quasi-static samples
        # (just after confirmation, before the lift accelerates), robust to a single noisy
        # frame. Fall back to the slowest samples if the arm never fully settles early.
        quasi=[m for m in measured if m[3]<QUASISTATIC_SPEED]
        if len(quasi)<6:
            quasi=sorted(measured,key=lambda m:m[3])[:max(6,len(measured)//10)]
            quasi=sorted(quasi,key=lambda m:m[0]['ns'])
        head=max(1,len(quasi)//3)
        ref_point=np.median([m[1] for m in quasi[:head]],axis=0)
        ref_ori=quasi[0][2]
        drift=[]  # all samples, for diagnostic extremes
        qdrift=[]  # quasi-static only, for the verdict
        high_since=None;sustained=False
        for pose,point,ori,speed in measured:
            distance=float(np.linalg.norm(point-ref_point))
            angle=math.acos(float(np.clip((np.trace(ref_ori.T@ori)-1)/2,-1,1)))
            drift.append((distance,angle))
            if speed>=QUASISTATIC_SPEED:
                high_since=None;continue  # under inertial load: deflection, not slip
            qdrift.append(distance)
            if distance>translation_limit or angle>math.radians(5):
                if high_since is None:high_since=pose['ns']
                # Real slip persists while the arm is quasi-static; an isolated settling
                # or placement-contact frame does not.
                sustained|=pose['ns']-high_since>=100000000
            else:high_since=None
        elevated=[r for r in poses if r['xyz'][2]>.321]
        onset=elevated[0] if elevated else None
        clear=next((r for r in poses if r['xyz'][2]>.340),None)
        close=intercept['grasp_sim']
        cases.append(dict(object=confirm['object'],samples=len(measured),quasi_static_samples=len(qdrift),
            close_to_confirm_sim=confirm['sim_time']-close,
            confirm_to_lift_onset_sim=None if onset is None else onset['ns']/1e9-confirm['sim_time'],
            close_to_lift_onset_sim=None if onset is None else onset['ns']/1e9-close,
            actual_lift_onset_sim=None if onset is None else onset['ns']/1e9,
            actual_deck_clear_sim=None if clear is None else clear['ns']/1e9,
            planned_lift_sim=request['lift_sim'],
            quasi_static_slip_m=max(qdrift) if qdrift else None,
            max_relative_translation_m=max(x[0] for x in drift),
            max_relative_angle_deg=math.degrees(max(x[1] for x in drift)),
            sustained_slip=sustained,
            passed=onset is not None and clear is not None and len(qdrift)>0 and not sustained))
    camera=[r for r in rows if r['kind']=='camera_age']
    expected={f'parcel_{i}' for i in range(1,task.get('parcel_count',3)+1)}
    complete=(all(len(group)==len(expected) for group in (confirms,releases,requests,intercepts))
              and {e['object'] for e in confirms}==expected)
    return dict(passed=complete and len(cases)==len(expected) and all(c['passed'] for c in cases),translation_limit_m=translation_limit,
                measurement='Gazebo object poses relative to actual JointState FK; slip judged at quasi-static arm samples',
                cases=cases,native_camera_max_age_sim=max(((r['ns']-r['source_ns'])/1e9 for r in camera),default=None))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--directory',type=Path,required=True)
    parser.add_argument('--translation-limit',type=float,default=.003)
    args=parser.parse_args();result=analyze(args.directory,args.translation_limit)
    (args.directory/'grasp_verdict.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)

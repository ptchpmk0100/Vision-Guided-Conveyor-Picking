#!/usr/bin/env python3
"""Model-only layout gate: bounded numeric IK, collision states and camera coverage.

No controller commands. This is plausibility evidence; real MoveIt execution and
Gazebo placement remain separate acceptance gates.
"""
import json
from pathlib import Path
import argparse
import math
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml
from moveit.core.robot_model import RobotModel
from moveit.core.planning_scene import PlanningScene
from twin_moveit_scripts.parcel_layout import batch_layout,PARCEL_SIZE
from twin_moveit_scripts.parcel_batch_world import write_batch_world
from twin_moveit_scripts.parcel_cell_scene import collision_objects
from twin_moveit_scripts.motion_audit import limit_profile,ARM_JOINTS
from twin_moveit_scripts.move_to import GRASP_SEED_JOINTS,TOPDOWN_GRASP_QUAT

ROOT=Path(__file__).resolve().parents[1]


def probe():
    batch=batch_layout(10)
    world=Path('/tmp/parcel_ten_gate.sdf')
    write_batch_world(ROOT/'src/twin_action_demo/worlds/industrial_parcel_cell_continuous.sdf',world,batch)
    urdf=ROOT/'src/twin_description/urdf/panda.urdf'
    model=RobotModel(str(urdf),str(ROOT/'src/twin_moveit_config/config/panda.srdf'))
    scene=PlanningScene(model)
    for obj in collision_objects(world):
        if scene.apply_collision_object(obj) is False:raise RuntimeError('Scene could not be installed')
    scene.allowed_collision_matrix.set_entry('panda_link0','robot_pedestal',True)
    state=scene.current_state
    state.set_joint_group_positions('hand',[.04,.04])
    limits=limit_profile(urdf.read_text(),yaml.safe_load((ROOT/'src/twin_moveit_config/config/joint_limits.yaml').read_text())['joint_limits'],.5,.5)
    low=np.array([limits[n]['position_min'] for n in ARM_JOINTS])
    high=np.array([limits[n]['position_max'] for n in ARM_JOINTS])
    target_rot=Rotation.from_quat(TOPDOWN_GRASP_QUAT).as_matrix()
    base=np.array([GRASP_SEED_JOINTS[n] for n in ARM_JOINTS]);previous=base.copy()
    results=[]
    for slot,(x,y) in enumerate(batch.slots):
        for height in (.61,.51,.66):
            target=np.array([x,y,height])
            def residual(q):
                state.set_joint_group_positions('panda_arm',q.tolist());state.update(force=True)
                tf=state.get_global_link_transform('panda_link8')
                return np.r_[tf[:3,3]-target,.2*Rotation.from_matrix(target_rot.T@tf[:3,:3]).as_rotvec()]
            seeds=[previous,base,base.copy()];seeds[-1][0]=math.atan2(y,x)
            good=None
            for seed in seeds:
                fit=least_squares(residual,np.clip(seed,low+1e-6,high-1e-6),bounds=(low,high),
                                  max_nfev=300,ftol=1e-10,xtol=1e-10,gtol=1e-10)
                error=residual(fit.x)
                collision=scene.is_state_colliding(robot_state=state,joint_model_group_name='')
                if np.linalg.norm(error[:3])<.001 and np.linalg.norm(error[3:])<.002 and not collision:
                    good=fit.x;break
            results.append(dict(slot=slot,target=target.tolist(),passed=good is not None,
                                position_error=float(np.linalg.norm(error[:3])),collision=bool(collision)))
            if good is not None:previous=good
    transform=yaml.safe_load((ROOT/'src/twin_action_demo/config/overhead_camera_extrinsic.yaml').read_text())['transform']
    r=np.array(transform['rotation_matrix']);t=np.array(transform['translation'])
    f=640/(2*math.tan(batch.camera_hfov/2));pixels=[]
    for x,y in batch.slots:
        for dx in (-.04,.04):
            for dy in (-.025,.025):
                p=r.T@(np.array([x+dx,y+dy,.345])-t)
                pixels.append([320+f*p[0]/p[2],240+f*p[1]/p[2]])
    visible=all(5<=u<=635 and 5<=v<=475 for u,v in pixels)
    return dict(passed=all(p['passed'] for p in results) and visible,ik_states=results,
                camera_all_corners_visible=visible,pixel_bounds=[np.min(pixels,axis=0).tolist(),np.max(pixels,axis=0).tolist()],
                execution_authorized=False,note='Model-only feasibility, not actual-motion or perception proof')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=probe();args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'],states=len(result['ik_states']),failed=[r for r in result['ik_states'] if not r['passed']],camera=result['camera_all_corners_visible'])))
    raise SystemExit(0 if result['passed'] else 1)

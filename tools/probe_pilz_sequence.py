#!/usr/bin/env python3
"""Isolated planning-only sequence server, explicit scene sync, zero/small blend.

No Gazebo or actual robot motion. Starts are declared ready-pose planning fixtures,
not measured starts and must never be used for execution. Repeats a fresh server
per run. Validates final sampled bounds/collision with an independent local scene.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import get_logger
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState
from moveit.core.planning_scene import PlanningScene
from moveit.utils import create_params_file_from_dict
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_msgs.msg import Constraints, JointConstraint, MotionSequenceItem, RobotState as StateMsg
from sensor_msgs.msg import JointState
from rcl_interfaces.srv import GetParameters
from twin_moveit_scripts import motion_audit
from twin_moveit_scripts.parcel_cell_scene import collision_objects
from twin_moveit_scripts.pilz_sequence import PilzPlanningClient

ROOT = Path(__file__).resolve().parents[1]


def probe(directory, number):
    namespace = f'/parcel_pilz_probe_{number}'
    mc = MoveItConfigsBuilder('panda',package_name='twin_moveit_config').planning_pipelines(
        pipelines=['pilz_industrial_motion_planner'],
        default_planning_pipeline='pilz_industrial_motion_planner').to_moveit_configs()
    cfg = mc.to_dict()
    cfg['planning_pipelines'] = {'pipeline_names':['pilz_industrial_motion_planner']}
    cfg['capabilities'] = 'pilz_industrial_motion_planner/MoveGroupSequenceService'
    cfg['disable_capabilities'] = ('move_group/MoveGroupExecuteTrajectoryAction '
        'move_group/MoveGroupMoveAction pilz_industrial_motion_planner/MoveGroupSequenceAction')
    cfg['allow_trajectory_execution'] = False
    cfg['use_sim_time'] = False
    params = create_params_file_from_dict(cfg,'/**')
    limits = motion_audit.limit_profile(cfg['robot_description'],
        cfg['robot_description_planning']['joint_limits'],.5,.5)
    model = RobotModel(str(ROOT/'src/twin_description/urdf/panda.urdf'),
                       str(ROOT/'src/twin_moveit_config/config/panda.srdf'))
    local = PlanningScene(model)
    for obj in collision_objects():
        if local.apply_collision_object(obj) is False:
            raise RuntimeError('LOCAL_SCENE_FAILED')
    local.allowed_collision_matrix.set_entry('panda_link0','robot_pedestal',True)
    ready = [0.,-.785,0.,-2.356,0.,1.571,.785]
    state = local.current_state
    state.set_joint_group_positions('panda_arm',ready)
    state.set_joint_group_positions('hand',[.04,.04])
    state.update(force=True)
    if local.is_state_colliding(robot_state=state,joint_model_group_name=''):
        raise RuntimeError('DECLARED_START_IN_COLLISION')
    snapshot = local.planning_scene_message
    snapshot.is_diff = False
    names = motion_audit.ARM_JOINTS
    goal_states, xyz = [], []
    for delta in (.15,.30):
        goal = copy.deepcopy(state)
        q = ready.copy();q[0]+=delta
        goal.set_joint_group_positions('panda_arm',q);goal.update(force=True)
        goal_states.append(q)
        tf=goal.get_global_link_transform('panda_link8')
        xyz.append([float(tf[i][3]) for i in range(3)])
    env=os.environ.copy()
    marker='pilz_probe_'+uuid.uuid4().hex
    env['TWIN_PATH_SEQUENCE_OWNER']=marker
    log=(directory/f'pilz_server_{number}.log').open('w')
    executable='/opt/ros/jazzy/lib/moveit_ros_move_group/move_group'
    process=subprocess.Popen([executable,'--ros-args','--params-file',params,
        '-r',f'__ns:={namespace}'],env=env,stdout=log,stderr=subprocess.STDOUT)
    node=rclpy.create_node(f'pilz_probe_client_{number}')
    executor=SingleThreadedExecutor();executor.add_node(node)
    result=dict(server_pid=process.pid,owner=marker,namespace=namespace,
                execution_authorized=False,declared_start=ready,clock_domain='host_monotonic',cases=[])
    try:
        adapter=PilzPlanningClient(node,executor,namespace)
        revision=adapter.synchronize(snapshot)
        services=[name for name,types in node.get_service_names_and_types()
                  if name==namespace+'/move_group/get_parameters'
                  and 'rcl_interfaces/srv/GetParameters' in types]
        if len(services)!=1:raise RuntimeError('SERVER_EXECUTION_POLICY_UNVERIFIED')
        policy_client=node.create_client(GetParameters,services[0])
        policy=adapter.call(policy_client,GetParameters.Request(names=['allow_trajectory_execution']))
        if len(policy.values)!=1 or policy.values[0].type!=1 or policy.values[0].bool_value:
            raise RuntimeError('SERVER_EXECUTION_POLICY_UNSAFE')
        result['execution_disabled_verified']=True
        for radius in (0.,.005):
            items=[]
            for i,q in enumerate(goal_states):
                item=MotionSequenceItem()
                item.req.group_name='panda_arm'
                item.req.pipeline_id='pilz_industrial_motion_planner'
                item.req.planner_id='PTP'
                item.req.allowed_planning_time=5.
                item.req.max_velocity_scaling_factor=.5
                item.req.max_acceleration_scaling_factor=.5
                if i==0:
                    item.req.start_state=StateMsg(joint_state=JointState(
                        name=names+['panda_finger_joint1','panda_finger_joint2'],position=ready+[.04,.04]))
                item.req.goal_constraints=[Constraints(joint_constraints=[JointConstraint(
                    joint_name=n,position=float(v),tolerance_above=.0001,tolerance_below=.0001,weight=1.)
                    for n,v in zip(names,q)])]
                item.blend_radius=radius if i==0 else 0.
                items.append(item)
            started=time.monotonic()
            response=adapter.plan(items,xyz,[False,True],revision)
            verdicts=[]
            for message in response.planned_trajectories:
                data=motion_audit.trajectory_data(message)
                verdict=motion_audit.validate(data,limits,stationary_start=ready)
                if verdict['sampled_checks_passed']:
                    sample=copy.deepcopy(state)
                    for q in motion_audit.interpolated_positions(data):
                        sample.set_joint_group_positions('panda_arm',q);sample.update(force=True)
                        if local.is_state_colliding(robot_state=sample,joint_model_group_name=''):
                            verdict['sampled_checks_passed']=False;verdict['errors'].append('SAMPLED_COLLISION');break
                verdicts.append(dict(validation=verdict,metrics=motion_audit.metrics(data)))
            result['cases'].append(dict(blend_radius=radius,planning_wall_s=time.monotonic()-started,
                scene_revision=revision,trajectories=verdicts,
                passed=all(v['validation']['sampled_checks_passed'] for v in verdicts)))
        result['passed']=len(result['cases'])==2 and all(c['passed'] for c in result['cases'])
    except Exception as exc:
        result.update(passed=False,error=str(exc))
    finally:
        executor.remove_node(node);executor.shutdown(timeout_sec=0);node.destroy_node()
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGKILL):
            if process.poll() is not None:break
            try:
                values=Path(f'/proc/{process.pid}/environ').read_bytes().split(b'\0')
                if f'TWIN_PATH_SEQUENCE_OWNER={marker}'.encode() not in values:break
                process.send_signal(sig);process.wait(timeout=3)
            except subprocess.TimeoutExpired:continue
            except (FileNotFoundError,ProcessLookupError):break
        result['server_exit']=process.poll();log.close()
        result['planning_passed']=result.get('passed',False)
        result['server_health_errors']=[line for line in (directory/f'pilz_server_{number}.log').read_text().splitlines()
            if any(token in line for token in ('Segmentation fault','[FATAL]','[ERROR]','Traceback'))
            and 'No 3D sensor plugin' not in line]
        if result['server_exit']!=0 or result['server_health_errors']:
            result.update(passed=False,error='SERVER_HEALTH_FAILED')
        Path(params).unlink(missing_ok=True)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--runs',type=int,default=2)
    args=parser.parse_args()
    if not 1<=args.runs<=4:parser.error('--runs must be in [1,4]')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init()
    try:
        rows=[]
        for i in range(args.runs):
            rows.append(probe(args.output.parent,i))
            args.output.write_text(json.dumps(rows,indent=2)+'\n')
            print(json.dumps(rows[-1]),flush=True)
            if not rows[-1]['passed']:break
    finally:rclpy.shutdown()
    raise SystemExit(0 if len(rows)==args.runs and all(r['passed'] for r in rows) else 1)

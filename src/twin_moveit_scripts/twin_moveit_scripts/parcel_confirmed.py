"""Continuation trajectories for contact-confirmed conveyor lifting."""
import copy
import math
import time

from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from twin_moveit_scripts import move_to as motion
from twin_moveit_scripts.parcel_continuous import (following_path, duration, InterceptNotFeasible,
                                                   ORIGINAL_TIMING)
from twin_moveit_scripts.parcel_grasp import GraspEvaluator,close_command
from twin_moveit_scripts.parcel_cell_scene import carry_parcel,permit_payload_support


def stamp(value):
    t=duration(value)
    return Time(sec=t.sec,nanosec=t.nanosec)


def current_xyz(arm):
    arm.set_start_state_to_current_state()
    return tuple(map(float,arm.get_start_state().get_global_link_transform(motion.EE_LINK)[:3,3]))


def continuation(robot,arm,initial, target,velocity, original,lift_at,lift_time=1.3,grasp_time=1.6):
    # Re-send the complete common prefix with the ORIGINAL (now past) epoch.
    # Forgetting the old goal then samples the same path at the current time.
    trajectory=following_path(robot,arm,initial,target,velocity,total=lift_at+lift_time,
                              lift_at=lift_at,prefix=original,grasp_time=grasp_time)
    trajectory.header.stamp=copy.deepcopy(original.header.stamp)
    return trajectory


def handoff_probe(robot,arm,observed,event):
    initial=current_xyz(arm)
    target=(.56,-.13,.32)
    original=following_path(robot,arm,initial,target,.06,total=4.,no_lift=True)
    starts=observed.now()+.5
    original.header.stamp=stamp(starts)
    handles=[]
    rows=[]
    try:
        h=observed.wait(observed.arm_client.send_goal_async(FollowJointTrajectory.Goal(trajectory=original)))
        if not h.accepted:raise RuntimeError('Handover probe initial goal rejected')
        handles.append(h)
        observed.until(starts+1.8)
        lift_at=2.4
        replacement=continuation(robot,arm,initial,target,.06,original,lift_at)
        if observed.now()>starts+lift_at-.1:raise RuntimeError('Probe continuation planned too late')
        sent=observed.now()
        h=observed.wait(observed.arm_client.send_goal_async(FollowJointTrajectory.Goal(trajectory=replacement)))
        if not h.accepted:raise RuntimeError('Handover probe replacement rejected')
        handles.append(h)
        result=h.get_result_async()
        sampled=-1
        while not result.done():
            observed.poll(.01)
            now=observed.now()
            if now-sampled>=.02:
                rows.append([now,*current_xyz(arm)])
                sampled=now
        if result.result().result.error_code!=0:raise RuntimeError('Probe continuation execution failed')
        window=[r for r in rows if sent+.05<=r[0]<=starts+lift_at-.05]
        if len(window)<3:raise RuntimeError('Insufficient controller-handover evidence')
        errors=[abs(r[2]-(target[1]+.06*(r[0]-starts))) for r in window]
        speeds=[(b[2]-a[2])/(b[0]-a[0]) for a,b in zip(window,window[1:]) if b[0]>a[0]]
        passed=max(errors)<.008 and min(speeds)>.025
        event('HANDOFF_PROBE',passed=passed,max_follow_error=max(errors),min_forward_speed=min(speeds))
        if not passed:raise RuntimeError('Controller replacement introduced a stop/discontinuity')
        return dict(passed=True,sent_sim=sent,lift_sim=starts+lift_at,samples=rows,
                    max_follow_error=max(errors),min_forward_speed=min(speeds))
    except BaseException:
        for h in handles:observed.wait(h.cancel_goal_async(),check=False)
        raise


def intercept(robot,arm,logger,observed,track,velocity,event,timing=ORIGINAL_TIMING):
    if observed.contacts is None:raise RuntimeError('Confirmed mode requires pad sensors')
    observed.fingers()
    if min(observed.joints[n] for n in motion.FINGER_JOINTS)<.035:
        raise RuntimeError('Both fingers must be open before interception')
    initial=current_xyz(arm)
    starts=observed.now()+timing.lead
    target=track.predict(starts,velocity)
    if target[1]+velocity*4.>.24:raise RuntimeError('Insufficient reach for confirmed interception')
    original=following_path(robot,arm,initial,target,velocity,total=4.,no_lift=True,
                            grasp_time=timing.grasp)
    if starts-observed.now()<.15:raise RuntimeError('Confirmed approach planning missed lead time')
    original.header.stamp=stamp(starts)
    close_start=starts+timing.grasp
    fingers=JointTrajectory(joint_names=list(motion.FINGER_JOINTS))
    fingers.header.stamp=stamp(close_start)
    fingers.points=[JointTrajectoryPoint(positions=[.04,.04],time_from_start=duration(0)),
                    JointTrajectoryPoint(positions=[.020,.020],time_from_start=duration(1.))]
    handles=[]
    evaluator=GraspEvaluator()
    previous=(False,False)
    try:
        for client,trajectory in ((observed.arm_client,original),(observed.finger_client,fingers)):
            h=observed.wait(client.send_goal_async(FollowJointTrajectory.Goal(trajectory=trajectory)))
            if not h.accepted:raise RuntimeError('Confirmed interception goal rejected')
            handles.append(h)
        event('INTERCEPT',track=track.number,velocity=velocity,measured_velocity=track.estimate(),
              camera_stamp=track.samples[-1][0],target_at_start=list(target),start_sim=starts,
              grasp_sim=close_start,end_sim=starts+4.,grasp_mode='confirmed')
        observed.until(close_start)
        event('CLOSE_START',scheduled_sim=close_start)
        while observed.now()<close_start+1.55:
            observed.poll(.01)
            sample=observed.grip_sample(close_command(observed.joint_stamp_ns/1e9-close_start))
            pair=sample.left&sample.right
            current=(bool(sample.left or sample.right),len(pair)==1)
            if current[0] and not previous[0]:event('FIRST_CONTACT',left=sorted(sample.left),right=sorted(sample.right))
            if current[1] and not previous[1]:event('BILATERAL_CONTACT',objects=sorted(pair))
            previous=current
            ready,reason=evaluator.update(sample)
            if ready:break
        else:raise RuntimeError('Grasp not confirmed before reachable following deadline: '+evaluator.reason)
        width=sum(sample.positions)/2
        event('GRASP_CONFIRMED',object=evaluator.object_id,width=width,desired=sample.desired,
              joint_age=sample.joint_age,contact_age=sample.contact_age,reason=reason)
        motion._record_grasp(width,logger)
        observed.monitor_grasp(evaluator,close_start)
        carry_parcel(robot)
        observed.payload_attached=True
        permit_payload_support(robot,True)
        # A past-epoch full continuation avoids an intervening hold caused by a
        # future-only goal. The common prefix is byte-identical q/v waypoints.
        lift_at=math.ceil((observed.now()-starts+.15)/.05)*.05
        try:
            replacement=continuation(robot,arm,initial,target,velocity,original,lift_at,
                                     timing.lift,timing.grasp)
        except InterceptNotFeasible as exc:
            # Controller goals are already running: this is a fault, never a retry.
            raise RuntimeError('Confirmed lift continuation infeasible: '+str(exc)) from exc
        if observed.now()>starts+lift_at-.05:raise RuntimeError('Confirmed lift planning missed handover lead')
        h=observed.wait(observed.arm_client.send_goal_async(FollowJointTrajectory.Goal(trajectory=replacement)))
        if not h.accepted:raise RuntimeError('Confirmed lift replacement rejected')
        handles.append(h)
        event('LIFT_REQUESTED',object=evaluator.object_id,lift_sim=starts+lift_at,
              close_start_sim=close_start,end_sim=starts+lift_at+timing.lift)
        result=h.get_result_async()
        support_cleared=False
        while not result.done():
            observed.poll(.01)
            if not support_cleared and current_xyz(arm)[2]>.445:
                permit_payload_support(robot,False)
                support_cleared=True
        if result.result().result.error_code!=0:raise RuntimeError('Confirmed lift execution failed')
        observed.check()
        actual=current_xyz(arm)
        from twin_moveit_scripts.parcel_tracking import follow_distance
        end=lift_at+timing.lift
        expected=(target[0],target[1]+follow_distance(end,velocity,end),target[2]+.25)
        if math.dist(actual,expected)>.012:raise RuntimeError('Confirmed lift did not arrive')
        retained=observed.fingers()
        if retained-.020<=.0015:raise RuntimeError('Empty hand after confirmed lift')
        event('LIFTED',object=evaluator.object_id,width=retained,position=list(actual))
    except BaseException:
        observed.stop()
        observed.carry_monitor=None
        observed.io.send('release')
        for h in handles:observed.wait(h.cancel_goal_async(),check=False)
        raise

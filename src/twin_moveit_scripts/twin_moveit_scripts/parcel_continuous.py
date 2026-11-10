"""Continuous conveyor following. Runtime targets come only from calibrated RGBD."""
import math
import copy
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from control_msgs.action import FollowJointTrajectory
from action_msgs.srv import CancelGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from moveit.core.robot_state import RobotState

from twin_moveit_scripts import move_to as motion
from twin_moveit_scripts.parcel_tracking import BeltTracker, follow_distance
from twin_moveit_scripts.parcel_layout import MOVING_DETECTION_TOPIC

ALIGN_TIME = .8
# Extended (7/8/10-box) profiles launch as soon as a track is mature and reachable,
# not only once the box is 0.16 m before the original launch point.
EARLY_LAUNCH_Y = -.250


class InterceptNotFeasible(RuntimeError):
    """Raised BEFORE any controller goal is sent: the planned follow path is not
    (yet) executable. The caller may retry once the box is closer."""
GRASP_TIME = 1.6
VERIFY_TIME = 3.1
LIFT_TIME = 3.2
TOTAL_TIME = 4.5


class Timing:
    """Interception timeline (simulated seconds). The defaults are the original,
    conservative three-box values; FAST_TIMING is used by the 7/8/10-box profiles."""
    def __init__(self, lead=.5, grasp=1.6, lift=1.3):
        self.lead = lead      # planning lead before the arm starts to move
        self.grasp = grasp    # fingers start closing; descent ends here
        self.lift = lift      # duration of the lift after the grasp is confirmed


ORIGINAL_TIMING = Timing()
FAST_TIMING = Timing(lead=.3, grasp=1.35, lift=.9)


def duration(value):
    ns = round(value*1e9)
    return Duration(sec=ns//1000000000, nanosec=ns % 1000000000)


def background_observations(observation_class, speed, sensors=False):
    """Use a separate node: MoveIt/gripper helpers retain ownership of twin's executor."""
    class ContinuousObservations(observation_class):
        def __init__(self):
            self.control = Node('parcel_continuous_control', parameter_overrides=[
                Parameter('use_sim_time', value=True)])
            super().__init__(self.control, detection_topic=MOVING_DETECTION_TOPIC,
                             region=lambda x, y: .505 <= x <= .615 and -.4 <= y <= .18)
            self.speed = 0.0
            self.fault = None
            self.joints = None
            self.joints_at = 0.0
            self.joint_stamp_ns = 0
            self.velocities = {}
            self.contacts = None
            self.carry_monitor = None
            self.payload_attached = False
            from twin_moveit_scripts.parcel_io import IsolatedIO
            self.io=IsolatedIO(sensors)
            self.io_state={}
            if sensors:
                class RemoteContacts:
                    packets={}
                    def sample(inner,now):
                        if len(inner.packets)!=2:return frozenset(),frozenset(),float('inf')
                        a,b=inner.packets['left'],inner.packets['right']
                        age=max((now-p['stamp_ns'])/1e9 for p in (a,b))
                        # Sim-time age (above) is the meaningful freshness gate. The wall
                        # guard only exists to reject a frozen sensor stream, so it is sized
                        # to tolerate low GUI real-time factor (~0.3) rather than the old
                        # .5s budget, which flagged sim-fresh contacts as stale under the GUI.
                        if any(time.monotonic()-p['received_wall']>1.5 for p in (a,b)):age=float('inf')
                        return a['parcels'],b['parcels'],age
                self.contacts = RemoteContacts()
            self.width_history = deque(maxlen=300)
            self.closing=False
            self.pending_slot=None
            self.tracker=None
            self.tracker_lock=threading.Lock()
            self.control.create_timer(.05, self.heartbeat, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.arm_client = ActionClient(self.control, FollowJointTrajectory,
                                           '/arm_controller/follow_joint_trajectory')
            self.finger_client = ActionClient(self.control, FollowJointTrajectory, motion.GRIPPER_ACTION)
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.control)
            self.thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.thread.start()

        def callback(self,msg):pass  # the isolated I/O process owns sensor inputs
        def tote_callback(self,msg):pass

        def sync(self):
            data=self.io.snapshot()
            if data is None:return
            self.io_state=data
            for name in ('message','received','tote_message','joints','joints_at','joint_stamp_ns','velocities'):
                setattr(self,name,data[name])
            if data['fault'] is not None:self.fault=data['fault']
            self.width_history=deque(data['width_history'],maxlen=1000)
            if self.contacts:self.contacts.packets=data['packets']

        def heartbeat(self):
            if not self.closing:
                self.sync()
                self._track()

        def enable_tracking(self, belt_speed):
            """Keep ONE camera-only tracker fed for the whole run (extended profiles)."""
            with self.tracker_lock:
                self.tracker = BeltTracker(belt_speed)

        def _track(self):
            if self.tracker is None:
                return
            try:
                positions = super().current()
            except RuntimeError:
                return          # e.g. a carried/lifted box off the deck: skip this sample
            stamp = self.source_stamp()
            if positions is None or stamp is None:
                return
            with self.tracker_lock:
                self.tracker.update(stamp, positions)

        def tracked_target(self, now, hand_xyz, window):
            with self.tracker_lock:
                return self.tracker.nearest(now, hand_xyz, window)

        def poll(self, seconds=.15, speed=None):
            # The periodic publisher stays alive during blocking arm execution.
            time.sleep(seconds)
            self.check()

        def check(self):
            self.sync()
            if not self.thread.is_alive():
                self.stop()
                raise RuntimeError('Continuous observation executor stopped')
            if self.fault:
                raise RuntimeError(self.fault)
            self._service_slot()

        # Non-blocking tote-slot confirmation: the same evidence as confirm_slot
        # (a fresh depth observation within 15 mm of the slot, stable for 0.6 s) but
        # gathered while the arm already heads for the next box. A gap in samples
        # restarts the stability window, and the next placement waits for it.
        def begin_slot_confirmation(self, x, y, confirmed=None):
            if self.pending_slot is not None:
                raise RuntimeError('Previous tote slot is still unconfirmed')
            from twin_moveit_scripts.tote_slot import SlotConfirmation
            self.pending_slot = (SlotConfirmation(x, y, self.now()), confirmed)

        def _service_slot(self):
            if self.pending_slot is None:
                return
            rule, confirmed = self.pending_slot
            valid, stamp = self._slot_sample(rule.x, rule.y)
            try:
                done = rule.update(valid, stamp, self.now())
            except RuntimeError:
                self.pending_slot = None
                raise
            if done:
                self.pending_slot = None
                if confirmed:
                    confirmed()

        def settle_pending_slot(self):
            while self.pending_slot is not None:
                self.poll(.02)

        def now(self):
            self.sync()
            return self.io_state.get('now_ns',0)/1e9

        def health(self):
            self.sync()
            stamp=None if self.message is None else self.message.header.stamp.sec*1000000000+self.message.header.stamp.nanosec
            return dict(received_age_wall=time.monotonic()-self.received,
                        source_age_sim=None if stamp is None else (self.io_state['now_ns']-stamp)/1e9)

        def current(self):
            self.sync()
            return super().current()

        def start(self):
            deadline = time.monotonic()+8
            while time.monotonic() < deadline:
                self.check()
                if self.current() is not None and self.pub.get_subscription_count():
                    self.speed = speed
                    self.io.send('speed',speed)
                    return
                self.poll(.02)
            raise RuntimeError('Continuous feed requires fresh perception and a feed guard')

        def require_empty_tote(self):
            # Observations have accumulated during approach/open. Accept fresh
            # feedback immediately instead of discarding it and sleeping a second.
            deadline = time.monotonic()+8
            while time.monotonic() < deadline:
                self.check()
                msg = self.tote_message
                if msg is not None:
                    stamp_ns = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
                    age = (self.io_state['now_ns']-stamp_ns)/1e9
                    if msg.header.frame_id == 'world' and -.05 <= age <= .7:
                        if msg.detections:
                            raise RuntimeError('Tote is occupied; empty/reset it before a new batch')
                        return
                self.poll(.02)
            raise RuntimeError('No fresh empty-tote observation; feed inhibited')

        def stop(self):
            self.speed = 0.0
            self.io.send('speed',0.)
            self.pub.publish(Float64(data=0.0))

        def close(self):
            self.closing=True
            self.stop()
            time.sleep(.15)
            self.executor.shutdown(timeout_sec=2)
            self.thread.join(timeout=2)
            self.control.destroy_node()
            self.io.close()

        def grip_sample(self, desired):
            from twin_moveit_scripts.parcel_grasp import GripSample
            self.sync()
            now = self.io_state['now_ns']
            left, right, age = self.contacts.sample(now) if self.contacts else (frozenset(), frozenset(), float('inf'))
            return GripSample(self.joint_stamp_ns, tuple(self.joints[name] for name in motion.FINGER_JOINTS),
                              desired, left, right, (now-self.joint_stamp_ns)/1e9, age)

        def release_grasp(self):
            self.check()
            self.carry_monitor=None
            self.io.send('release')

        def monitor_grasp(self,evaluator,close_start):
            self.carry_monitor=(evaluator,close_start)
            self.io.send('watch',(evaluator.object_id,close_start))

        def wait(self, future, timeout=30, check=True):
            deadline = time.monotonic()+timeout
            while not future.done() and time.monotonic() < deadline:
                time.sleep(.01)
                if check:
                    self.check()
            if not future.done():
                raise RuntimeError('Continuous controller action timed out')
            return future.result()

        def until(self, when, timeout=40):
            deadline = time.monotonic()+timeout
            while self.now() < when:
                self.poll(.01)
                if time.monotonic() > deadline:
                    raise RuntimeError('Simulation clock did not advance during interception')

        def fingers(self):
            self.sync()
            if self.joints is None or time.monotonic()-self.joints_at > .5:
                raise RuntimeError('No fresh joint feedback for continuous grasp')
            widths = [self.joints[name] for name in motion.FINGER_JOINTS]
            # Independent position-driven fingers can see-saw around a held box.
            # The gap (sum), as in move_to._on_joint_states, measures seating;
            # equality of the individual joints does not measure retained grasp.
            return sum(widths)/2

    return ContinuousObservations()


def prepare(robot, arm, logger, params, observed, event, startup=False):
    for client in (observed.arm_client, observed.finger_client):
        if not client.wait_for_server(timeout_sec=8):
            raise RuntimeError('Controller unavailable for continuous preparation')
    deadline = time.monotonic()+8
    while observed.joints is None or time.monotonic()-observed.joints_at > .5:
        observed.poll(.02)
        if time.monotonic() >= deadline:
            raise RuntimeError('No fresh joint feedback for continuous preparation')

    def stroke(seconds):
        trajectory = JointTrajectory(joint_names=list(motion.FINGER_JOINTS), points=[
            JointTrajectoryPoint(positions=[motion.GRIPPER_MAX]*2, time_from_start=duration(seconds))])
        handle = observed.wait(observed.finger_client.send_goal_async(
            FollowJointTrajectory.Goal(trajectory=trajectory)))
        if handle is None or not handle.accepted:
            raise RuntimeError('Preparation open rejected by gripper controller')
        try:
            return observed.wait(handle.get_result_async())
        except BaseException:
            observed.wait(handle.cancel_goal_async(), check=False)
            raise

    def open_fingers():
        event('PREPARE_OPEN_START', startup=startup)
        if startup:
            # The simulator can swallow its first finger command. Burn it while
            # the arm travels, then verify the real opening from both joint values.
            stroke(.2)
        for attempt in range(motion.GRIPPER_OPEN_ATTEMPTS+1):
            observed.fingers()  # fresh feedback required even when skipping a stroke
            if min(observed.joints[name] for name in motion.FINGER_JOINTS) >= (
                    motion.GRIPPER_MAX-motion.GRIPPER_OPEN_TOL):
                motion._clear_grasp(logger, 'verified empty open hand for continuous approach')
                event('PREPARE_OPEN_DONE', startup=startup, strokes=attempt,
                      fingers=[observed.joints[name] for name in motion.FINGER_JOINTS])
                return
            if attempt < motion.GRIPPER_OPEN_ATTEMPTS:
                stroke(motion.GRIPPER_MOVE_TIME)
        raise RuntimeError('Both fingers did not open for continuous preparation')

    # Arm execution and finger feedback use separate ROS executors. The worker
    # owns only gripper actions; all arm planning remains on this calling thread.
    with ThreadPoolExecutor(max_workers=1) as pool:
        opening = pool.submit(open_fingers)
        event('PREPARE_ARM_START', startup=startup)
        ready = motion.move_to_grasp_pose(robot, arm, logger, params, .56, .060, .520,
                                         motion.TOPDOWN_GRASP_QUAT)
        event('PREPARE_ARM_DONE', startup=startup, arrived=ready.ok)
        opening.result()
    if not ready.ok:
        raise RuntimeError(f'Cannot reach continuous observation approach: {ready}')


def next_target(observed, speed, timeout=None, window=None):
    persistent = window is not None and getattr(observed, 'tracker', None) is not None
    tracker = BeltTracker(speed)
    if timeout is None:
        timeout = max(40., 1.2/speed+10.)
    deadline = observed.now()+timeout
    wall_deadline = time.monotonic()+timeout*5
    report_at = 0
    track_report_stamp = None
    while observed.now() < deadline and time.monotonic() < wall_deadline:
        observed.poll(.025)
        if persistent:
            found = observed.tracked_target(observed.now(), (.56, .06, .52), window)
            if found:
                return found
            if time.monotonic() >= report_at:
                observed.node.get_logger().info(f'CONTINUOUS_WAIT speed={speed:.3f} sim_time={observed.now():.3f}')
                report_at = time.monotonic()+5
            continue
        positions = observed.current()
        if positions is not None:
            tracker.update(observed.source_stamp(), positions)
            found = tracker.nearest(observed.now(), (.56, .06, .52))
            if found:
                return found
            if tracker.last_stamp != track_report_stamp:
                for track in tracker.tracks:
                    estimate = track.estimate()
                    if estimate is not None and abs(estimate-speed) > .010:
                        observed.node.get_logger().info('TRACK_WAIT uncertain_velocity=' + str(estimate)
                            + ' camera_samples=' + str([(t, p[1]) for t, p in track.samples]))
                track_report_stamp = tracker.last_stamp
        if time.monotonic() >= report_at:
            observed.node.get_logger().info(f'CONTINUOUS_WAIT speed={speed:.3f} sim_time={observed.now():.3f}')
            report_at = time.monotonic()+5
    raise RuntimeError('Continuous arrival timeout: no reachable camera track')


def following_path(robot, arm, start_xyz, target_xyz, velocity, *, total=TOTAL_TIME,
                   lift_at=LIFT_TIME, no_lift=False, prefix=None, grasp_time=GRASP_TIME):
    """IK at 50 ms and collision-check intermediate joint states, without retiming.

    Explicit ROS simulation timestamps keep the Cartesian motion at measured belt
    velocity. Reject excessive joint motion rather than changing the intercept clock.
    """
    arm.set_start_state_to_current_state()
    state = arm.get_start_state()
    names = [f'panda_joint{i}' for i in range(1, 8)]
    initial = np.array(state.get_joint_group_positions(motion.ARM_GROUP))
    if prefix is not None:
        initial = np.array(prefix.points[0].positions)
        state.set_joint_group_positions(motion.ARM_GROUP, initial.tolist())
    previous = initial.copy()
    trajectory = JointTrajectory(joint_names=names)
    trajectory.points.append(JointTrajectoryPoint(positions=initial.tolist(), time_from_start=duration(0)))
    descend = grasp_time
    for index in range(1, round(total/.05)+1):
        t = index*.05
        blend = min(1., t/ALIGN_TIME)
        smooth = blend*blend*(3-2*blend)
        y = target_xyz[1]+follow_distance(t, velocity, total)+(start_xyz[1]-target_xyz[1])*(1-smooth)
        if t <= descend:
            # Match the moving parcel horizontally ABOVE it before descending.
            # A diagonal descent sweeps an open finger through the carton top.
            down = max(0., min(1., (t-ALIGN_TIME)/(descend-ALIGN_TIME)))
            z = start_xyz[2]+(target_xyz[2]+motion.FINGER_REACH_Z-start_xyz[2])*down*down*(3-2*down)
        else:
            lift = max(0., min(1., (t-lift_at)/(total-lift_at)))
            if no_lift:
                lift=0.
            z = target_xyz[2]+motion.FINGER_REACH_Z+.15*lift*lift*(3-2*lift)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = target_xyz[0], y, z
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = motion.TOPDOWN_GRASP_QUAT
        if prefix is not None and t <= lift_at+1e-9:
            state.set_joint_group_positions(motion.ARM_GROUP,prefix.points[index].positions)
        elif not state.set_from_ik(joint_model_group_name=motion.ARM_GROUP,
                                  geometry_pose=pose, tip_name=motion.EE_LINK, timeout=.03):
            raise InterceptNotFeasible('Continuous following IK failed')
        state.update(force=True)
        joints = np.array(state.get_joint_group_positions(motion.ARM_GROUP))
        if np.max(abs(joints-previous)/.05) > 1.0:
            raise InterceptNotFeasible('Continuous following exceeds joint velocity budget')
        with robot.get_planning_scene_monitor().read_only() as scene:
            for fraction in (.25, .5, .75, 1.):
                sample = copy.deepcopy(state)
                sample.set_joint_group_positions(motion.ARM_GROUP,
                    (previous+(joints-previous)*fraction).tolist())
                sample.set_joint_group_positions('hand', [.04, .04])
                sample.update(force=True)
                if scene.is_state_colliding(robot_state=sample, joint_model_group_name=''):
                    raise InterceptNotFeasible('Collision on continuous following path')
        trajectory.points.append(JointTrajectoryPoint(positions=joints.tolist(), time_from_start=duration(t)))
        previous = joints
    joints = np.array([p.positions for p in trajectory.points])
    velocities = np.gradient(joints, .05, axis=0)
    velocities[0] = 0.
    velocities[-1] = 0.
    if prefix is not None:
        for i in range(min(len(prefix.points),len(velocities))):
            if i*.05 <= lift_at+1e-9:
                velocities[i]=prefix.points[i].velocities
    # JTC uses cubic Hermite interpolation when positions+velocities are supplied.
    # Its acceleration is linear per interval, so check both exact extrema.
    displacement = np.diff(joints, axis=0)
    a0 = 6*displacement/.05**2-(4*velocities[:-1]+2*velocities[1:])/.05
    a1 = -6*displacement/.05**2+(2*velocities[:-1]+4*velocities[1:])/.05
    if max(np.max(abs(a0)), np.max(abs(a1))) > 7.5:
        peak = np.maximum(abs(a0), abs(a1))
        interval, joint = np.unravel_index(np.argmax(peak), peak.shape)
        raise InterceptNotFeasible(f'Continuous following exceeds joint acceleration budget: '
                           f'{peak[interval,joint]:.3f} rad/s2, t={interval*.05:.2f}, joint={joint+1}')
    for point, velocity in zip(trajectory.points, velocities):
        point.velocities = velocity.tolist()
    return trajectory


def intercept(robot, arm, logger, observed, track, velocity, event):
    observed.check()
    observed.fingers()  # require fresh feedback before submitting timed goals
    if min(observed.joints[name] for name in motion.FINGER_JOINTS) < .035:
        raise RuntimeError('Both fingers must be open before continuous interception')
    if not observed.arm_client.wait_for_server(timeout_sec=5) or not observed.finger_client.wait_for_server(timeout_sec=5):
        raise RuntimeError('Continuous controller action server unavailable')
    arm.set_start_state_to_current_state()
    transform = arm.get_start_state().get_global_link_transform(motion.EE_LINK)
    start_xyz = tuple(float(v) for v in transform[:3, 3])
    # Build first, then check the planning lead time before issuing any controller goal.
    starts = observed.now()+.5
    target = track.predict(starts, velocity)
    if target[1]+velocity*TOTAL_TIME > .24:
        raise RuntimeError('Insufficient reachable belt length for continuous intercept')
    trajectory = following_path(robot, arm, start_xyz, target, velocity)
    if starts-observed.now() < .15:
        raise RuntimeError('Continuous planning exceeded scheduled execution lead time')
    # Assign a Time message explicitly (Duration is a distinct ROS type).
    from builtin_interfaces.msg import Time
    stamp = duration(starts)
    trajectory.header.stamp = Time(sec=stamp.sec, nanosec=stamp.nanosec)
    arm_goal = FollowJointTrajectory.Goal(trajectory=trajectory)
    finger_traj = JointTrajectory(joint_names=list(motion.FINGER_JOINTS))
    finger_start = starts+GRASP_TIME
    stamp = duration(finger_start)
    finger_traj.header.stamp = Time(sec=stamp.sec, nanosec=stamp.nanosec)
    finger_traj.points = [JointTrajectoryPoint(positions=[.04, .04], time_from_start=duration(0)),
                         JointTrajectoryPoint(positions=[.020, .020], time_from_start=duration(1.0))]
    active = []
    try:
        for client, goal in ((observed.arm_client, arm_goal), (observed.finger_client,
                             FollowJointTrajectory.Goal(trajectory=finger_traj))):
            handle = observed.wait(client.send_goal_async(goal))
            if handle is None or not handle.accepted:
                raise RuntimeError('Continuous controller rejected a scheduled goal')
            active.append(handle)
        event('INTERCEPT', track=track.number, velocity=velocity, camera_stamp=track.samples[-1][0],
              measured_velocity=track.estimate(),
              target_at_start=list(target), start_sim=starts, end_sim=starts+TOTAL_TIME,
              grasp_sim=starts+GRASP_TIME, lift_sim=starts+LIFT_TIME)
        for offset in (ALIGN_TIME, GRASP_TIME, GRASP_TIME+1.):
            observed.until(starts+offset)
            arm.set_start_state_to_current_state()
            sample = arm.get_start_state().get_global_link_transform(motion.EE_LINK)[:3, 3]
            event('FOLLOWING', elapsed=offset, position=list(map(float, sample)),
                  fingers=[observed.joints[name] for name in motion.FINGER_JOINTS])
            if observed.contacts:
                s=observed.grip_sample(.020)
                event('PAD_CONTACTS', left=sorted(s.left), right=sorted(s.right), age=s.contact_age,
                      names={k:v['names'] for k,v in observed.contacts.packets.items()})
            if offset >= GRASP_TIME and math.dist(sample, (target[0], target[1]+velocity*offset,
                                                         target[2]+motion.FINGER_REACH_Z)) > .012:
                raise RuntimeError('Continuous descent failed to arrive at moving parcel')
        observed.until(starts+VERIFY_TIME)
        width = observed.fingers()
        recent = [w for t, w in list(observed.width_history)
                  if starts+VERIFY_TIME-.25 <= t <= starts+VERIFY_TIME]
        if not .019 <= width <= .031 or width-.020 <= motion.GRASP_HOLD_MIN:
            raise RuntimeError(f'Continuous grasp did not seat on parcel: width={width:.4f}')
        if len(recent) < 3 or max(recent)-min(recent) > .001:
            raise RuntimeError('Continuous grasp width is not stable before lift')
        motion._record_grasp(width, logger)
        result = observed.wait(active[0].get_result_async(), timeout=40)
        if result is None or result.result.error_code != 0:
            raise RuntimeError('Continuous arm following failed: '+str(result))
        # Controller result alone cannot establish arrival or retained grasp.
        arm.set_start_state_to_current_state()
        actual = arm.get_start_state().get_global_link_transform(motion.EE_LINK)[:3, 3]
        expected = (target[0], target[1]+follow_distance(TOTAL_TIME, velocity, TOTAL_TIME),
                    target[2]+motion.FINGER_REACH_Z+.15)
        if math.dist(actual, expected) > .012:
            raise RuntimeError('Continuous lift did not reach its predicted end pose')
        retained = observed.fingers()
        if (retained < width-motion.GRASP_DROP_MARGIN or not .019 <= retained <= .031
                or retained-.020 <= motion.GRASP_HOLD_MIN):
            motion._clear_grasp(logger, 'continuous lift lost the parcel')
            raise RuntimeError('Continuous parcel grasp lost during lift')
        event('LIFTED', width=retained, position=list(map(float, actual)))
    except BaseException:
        observed.stop()
        for handle in active:
            observed.wait(handle.cancel_goal_async(), check=False)
        raise

#!/usr/bin/env python3
"""Drive the Panda arm via moveit_py.

Usage:
  ros2 run twin_moveit_scripts move_to named ready
  ros2 run twin_moveit_scripts move_to pose 0.28 -0.2 0.5
  ros2 run twin_moveit_scripts move_to demo          # ready -> home

  ros2 run twin_moveit_scripts move_to grip 0.0        # close gripper
  ros2 run twin_moveit_scripts move_to grip 0.04       # open gripper

  ros2 run twin_moveit_scripts move_to pick box        # approach->grip->confirm->lift
  ros2 run twin_moveit_scripts move_to place table      # approach->descend->open->retract
  ros2 run twin_moveit_scripts move_to pick_and_place box table  # full cycle

Also importable: setup() -> (robot, arm, logger, params, twin);
move_to_named / move_to_pose / move_gripper / pick / place / pick_and_place; shutdown(twin).
"""
import os
import sys
import time
import copy
import math
from contextlib import contextmanager
from contextvars import ContextVar
from dataclasses import dataclass
from typing import Optional, Tuple
import rclpy

_MOTION_GUARD = ContextVar('parcel_motion_guard',default=None)


@contextmanager
def guarded_motion(check):
    token=_MOTION_GUARD.set(check)
    try:
        yield
    finally:
        _MOTION_GUARD.reset(token)


def _check_motion():
    check=_MOTION_GUARD.get()
    if check is not None:
        check()
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.robot_state import RobotState
from moveit.core.robot_trajectory import RobotTrajectory
from moveit.utils import create_params_file_from_dict
from moveit_configs_utils import MoveItConfigsBuilder
from twin_moveit_scripts import motion_audit

_MOTION_LIMITS = None
_MOTION_PROFILE_HASH = None
_REQUEST_OPTIONS = None
_EXPERIMENT_PIPELINE = 'ompl'
_FALLBACK_PARAMS = None
_SHADOW_PROFILE = None
_SHADOW_NODE = None
_SHADOW_EXECUTOR = None
_SHADOW_COMMAND = None


def _extended_profile():
    """True for the 7/8/10-box profiles; the original three-box path stays unchanged."""
    return os.environ.get('TWIN_PARCEL_COUNT') in ('7', '8', '10')


def _retime_margin():
    # Reserve 1% inside the unchanged operating envelope for the multi-box profiles.
    # The first ten-box pilot exposed TOTG's small numerical acceleration overshoot.
    return .99 if _extended_profile() else 1.

ARM_GROUP = "panda_arm"
EE_LINK = "panda_link8"
PLANNING_FRAME = "panda_link0"

# ---- Gripper (Session 15/16, rewired S17 to JointTrajectoryController) ----
# Was a GripperActionController on panda_finger_joint1 alone. DART can't enforce
# the URDF mimic, so the right finger was uncoupled (asymmetric close, friction
# grip, oscillation). Now a JointTrajectoryController drives BOTH finger joints
# together via FollowJointTrajectory. The per-finger `position` is the prismatic
# joint VALUE in meters (single-finger travel), URDF limit 0.0 (closed)..0.04
# (open). Both fingers get the same commanded value each point -> symmetric close.
GRIPPER_ACTION = "/gripper_controller/follow_joint_trajectory"
FINGER_JOINT = "panda_finger_joint1"       # (S21: no longer the width source; width is the mean of FINGER_JOINTS)
FINGER_JOINTS = ["panda_finger_joint1", "panda_finger_joint2"]  # both commanded
GRIPPER_MIN = 0.0
GRIPPER_MAX = 0.04
# Time budget for the finger trajectory to execute (s). The prismatic joints are
# fast (URDF velocity limit 0.2 m/s over 0.04 m of travel), so this is generous.
GRIPPER_MOVE_TIME = 2.5
# S19: finger reads were being taken mid-stroke. Wait for the width to STOP
# CHANGING, not for a fixed duration.
# S19b: the OPEN after a grasp is the slow case -- a position-mode JTC has to
# reverse a settled hold-command (0.0101) and travel the full 30mm, which took
# >3s in Gazebo. MOVE_TIME (trajectory duration) and SETTLE_MAX (how long we
# watch) both had to grow to cover it; a 1.5s trajectory simply couldn't get
# there. These are the levers if the open still doesn't complete.
GRIPPER_SETTLE_MAX = 6.0     # hard cap; must exceed the loaded-open stroke time
GRIPPER_SETTLE_MIN = 1.2     # minimum watch time; never believe stillness before
                             # this floor (defeats the stale-read trip)
GRIPPER_SETTLE_DT = 0.15     # pace between samples so prev/w are distinct msgs
GRIPPER_SETTLE_TOL = 0.0005  # 0.5 mm: reads this close count as "not moving"
GRIPPER_SETTLE_HOLD = 0.3    # must stay still this long to be called settled
# Grasp detection (replaces the controller's old `stalled` flag, which gave false
# HELDs). On a CLOSING command, if the fingers stop meaningfully short of the
# commanded close, something is between them -> held. Object-size-agnostic: it
# measures the intent-vs-outcome gap, not an absolute width. This margin (m) is
# the noise/friction floor below which a short-stop is not a real grasp.
GRASP_GAP_MIN = 0.003
# S20: GRASP_GAP_MIN used to do THREE unrelated jobs -- detect-stroke floor (above),
# hold verification, and place()'s "is the hand empty" test. One number tuned for
# one job silently set the other two. Split, each named for its own job:
#
# GRASP_HOLD_MIN: how much compression must REMAIN at the end of the hold stroke
# for the grasp to be real. A genuine hold settles ~GRASP_SQUEEZE wider than
# hold_target (the object blocks the squeeze); a spurious one lands ON target.
# The usable headroom is (GRASP_SQUEEZE - GRASP_HOLD_MIN): at the old shared
# 0.003 that was only 2 mm, so an object compressing ~2 mm elastically would have
# had a REAL grasp declared spurious. 0.0015 gives 3.5 mm of headroom while
# staying 3x above GRIPPER_SETTLE_TOL (read noise). INVARIANT, do not break:
#   GRIPPER_SETTLE_TOL < GRASP_HOLD_MIN < GRASP_SQUEEZE
GRASP_HOLD_MIN = 0.0015
# GRIPPER_EMPTY_MARGIN: fingers within this of GRIPPER_MAX are commanded-open, so
# nothing can be pinched. Only used as the LAST-RESORT test in place() when no
# grasp was recorded in this process (see _GRASP_STATE).
GRIPPER_EMPTY_MARGIN = 0.003
# A3: how far below an OPENING command the fingers may settle before we call the open
# failed and retry. The first gripper command after bringup is sometimes silently
# dropped -- the controller accepts the goal but the fingers never leave their 0.02
# spawn position, and move_gripper happily reported "ok [reached] width=0.0200" against
# a 0.0400 command. A half-open hand (0.04 total) then fouls the descend onto a 3 cm
# object: the arm stalls ~29 mm high, the object gets nudged, and pick fails at
# descend_failed. Observed in BOTH collision-on and collision-off runs, so it is a
# pre-existing start-up race, not a planning-scene problem. S21 retried arm execution
# but never gripper commands; this closes that gap. Only applied to OPENING strokes --
# on a CLOSE, stopping short IS the grasp signal.
GRIPPER_OPEN_TOL = 0.005
# 3, not 2: an immediate re-send still landed at 0.0200 because it raced the same
# not-yet-ready controller. Retries now wait GRIPPER_OPEN_RETRY_WAIT first.
GRIPPER_OPEN_ATTEMPTS = 3
GRIPPER_OPEN_RETRY_WAIT = 1.5
# Two-stage grasp hold (S17): commanding the fingers to 0.0 against a rigid object
# they can't reach leaves a huge standing position error, so the JTC keeps pushing
# inward forever -> the held object oscillates. Root fix: after the detect stroke
# finds the object at width w, RE-COMMAND the fingers to (w - GRASP_SQUEEZE) --
# just inside the surface, a target that's reachable but for a small elastic
# margin. That gives a firm hold with a settled, near-zero standing error instead
# of an endless push. Squeeze is the holding penetration (m); small = gentle.
GRASP_SQUEEZE = 0.005
# Post-lift grasp re-verify tolerance (m). After lifting, if the fingers have
# closed more than this below where the object seated them at grasp time, the
# object slipped out mid-lift -> pick reports 'dropped_in_lift', not success.
GRASP_DROP_MARGIN = 0.008
# S19b: on OPEN, a release is confirmed when the fingers open at least this far
# PAST the grasp width. Relative test tolerates the loaded open stroke finishing
# short of a hard 0.04. A ~15mm cube held at ~0.015 releasing to ~0.031+ opens
# ~0.016, comfortably above this margin; a still-pinched object barely moves.
GRASP_RELEASE_MARGIN = 0.008
GRIPPER_DEFAULT_EFFORT = 10.0

# ---- Grasp geometry for pick() (Session 17) ----
# Top-down grasp orientation for EE_LINK (panda_link8), as a (qx,qy,qz,qw) quat.
# Derived from two URDF facts:
#   1. Tool z (panda_link8 z) must point straight DOWN so the hand approaches
#      from above  -> a pi rotation about world x.
#   2. panda_hand is yawed -0.785398 (-pi/4) about z relative to panda_link8
#      (URDF panda_hand_joint). To land the HAND square with the world (so an
#      axis-aligned box meets the fingers on flat faces, not an edge), link8 must
#      carry +pi/4 to cancel that offset.
# Composed R = Rx(pi) * Rz(+pi/4); quaternion below. This is the deterministic
# replacement for last session's manual `-Y -0.785` box spawn: the ARM now owns
# the alignment, so the test cube spawns axis-aligned (no -Y).
TOPDOWN_GRASP_QUAT = (0.923880, -0.382683, 0.0, 0.0)  # (x, y, z, w)

# Depth of the finger PADS' center below panda_link8 along tool z. NOT the tip
# depth: the canonical panda_hand_tcp sits ~0.1034 below link8, but that puts the
# very fingertips at the target -- so the pads close ABOVE a low object. S17 run 1
# proved this: FINGER_REACH_Z=0.10 -> pad center 7 mm above a ground cube ->
# "finger above", stalled at width 0.0098 (empty). Pad center is ~0.093 below
# link8; command link8 to z_obj + FINGER_REACH_Z so the PADS straddle the object
# body. Tunable -- if the tips graze the ground, raise it a few mm.
# S21: was 0.093. finger.stl reaches 0.0538 below the finger frame, which is 0.0584
# below link8, so the tips are 0.112 below link8: at 0.093 the commanded z=0.108
# pressed the tips into the ground and the arm stopped at 0.112. 0.100 reaches its
# command (0.115) with ~3 mm of tip clearance and still seats the cube at 0.0151.
FINGER_REACH_Z = 0.100
# S21: seated half-width (mean finger position) of the 3 cm grasp_box, and how far a
# 'stalled' reading may sit from it before pick() refuses to believe it holds the cube.
CUBE_HALF_WIDTH = 0.015
GRASP_WIDTH_TOL = 0.006
# Extra height above the grasp pose for the pre-grasp approach and the lift.
APPROACH_CLEARANCE = 0.10
LIFT_HEIGHT = 0.15

# ---- Motion speed (Phase 1) ----------------------------------------------------
# The arm historically ran at 0.1 (10%) velocity AND acceleration scaling -- the
# MoveIt "beginner" default baked into both joint_limits.yaml and every request
# here. That one-tenth throttle is the primary reason pick/place looked slow.
# Scale toward the Panda's real joint limits for natural speed; override per run
# with TWIN_VEL_SCALE / TWIN_ACC_SCALE (no code edit) when tuning. Validate
# placement accuracy whenever raising these -- faster accel can overshoot.
VEL_SCALE = float(os.environ.get("TWIN_VEL_SCALE", "0.5"))
ACC_SCALE = float(os.environ.get("TWIN_ACC_SCALE", "0.5"))

# Post-execution settle (Phase 1). robot.execute() already BLOCKS until the
# controller finishes the trajectory, so the old fixed 3 s dwell after every
# segment was pure dead time (~30 s/parcel over ~10 segments). Instead confirm the
# arm has stopped moving by polling the live EE pose, with a short cap. See
# _settle_pose / _plan_execute.
SETTLE_CAP = float(os.environ.get("TWIN_SETTLE_CAP", "0.8"))  # max s to wait for stillness
SETTLE_DT = 0.05       # sample pacing (s)
SETTLE_TOL = 0.0015    # m: EE movement below this between samples counts as "still"
SETTLE_HOLD = 0.12     # s: must stay still this long to be called settled


# ---- Richer result from primitives (Session 14) ----
# Stable shape both move_to.py and llm_move.py agree on. Add fields here later
# (e.g. planning_time, distance_to_goal) without touching call sites.
@dataclass
class MoveResult:
    ok: bool                                     # planned AND executed without error
    stage: str                                   # "executed" | "plan_failed" | "exec_error" | "exec_incomplete"
    final_pose: Optional[Tuple[float, float, float]] = None  # live EE xyz after motion
    error: Optional[str] = None                  # human-readable reason when not ok

    def __str__(self):
        if self.ok and self.final_pose is not None:
            x, y, z = self.final_pose
            return f"ok @ ({x:.3f}, {y:.3f}, {z:.3f})"
        return f"FAILED [{self.stage}]: {self.error}"


# ---- Grip result (Session 16; stall now DERIVED, S17) ----
# `stalled` and `final_width` have no meaning for an arm move; `final_pose` has
# none for a grip. Keeping them separate keeps each result honest. CRITICAL:
# stalled==True is grasp SUCCESS (fingers stopped short of the commanded close =
# object between them). As of S17 this is DERIVED in move_gripper from the finger
# width vs commanded gap, NOT reported by the controller (JTC has no stall flag).
# pick() consumes exactly this signal.
@dataclass
class GripResult:
    ok: bool                                # goal accepted AND controller reported a terminal result
    stage: str                              # "reached" | "stalled" | "rejected" | "error"
    stalled: bool = False                   # True == held an object (grasp success), NOT failure
    final_width: Optional[float] = None     # live panda_finger_joint1 value after motion
    error: Optional[str] = None

    def __str__(self):
        if self.ok:
            w = f"{self.final_width:.4f}" if self.final_width is not None else "?"
            held = " HELD(stalled)" if self.stalled else ""
            return f"ok [{self.stage}] width={w}{held}"
        return f"FAILED [{self.stage}]: {self.error}"


def build_config_dict(logger):
    global _MOTION_LIMITS, _MOTION_PROFILE_HASH, _REQUEST_OPTIONS, _EXPERIMENT_PIPELINE
    global _SHADOW_PROFILE
    mc = MoveItConfigsBuilder("panda", package_name="twin_moveit_config").planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl").to_moveit_configs()
    d = mc.to_dict()
    names = d.get("planning_pipelines")
    if isinstance(names, list):
        d["planning_pipelines"] = {"pipeline_names": names}
    _REQUEST_OPTIONS = motion_audit.request_options(d['ompl'].get('planner_configs', {}))
    if os.environ.get('TWIN_MOTION_METRICS') or os.environ.get('TWIN_TRAJECTORY_CHECK'):
        _MOTION_LIMITS = motion_audit.limit_profile(d['robot_description'],
            d['robot_description_planning']['joint_limits'], VEL_SCALE, ACC_SCALE)
        _MOTION_PROFILE_HASH = motion_audit.digest(dict(limits=_MOTION_LIMITS,
            urdf=d['robot_description'], srdf=d['robot_description_semantic']))
    check = os.environ.get('TWIN_TRAJECTORY_CHECK', 'off')
    if check not in ('off', 'sampled'):
        raise ValueError(f'Unknown TWIN_TRAJECTORY_CHECK: {check}')
    _SHADOW_PROFILE = None
    if os.environ.get('TWIN_ARRIVAL_SHADOW'):
        import json
        from pathlib import Path
        from twin_moveit_scripts.arrival_gate import ArrivalProfile
        if not os.environ.get('TWIN_MOTION_METRICS'):
            raise ValueError('Arrival shadow requires TWIN_MOTION_METRICS output')
        _SHADOW_PROFILE = ArrivalProfile(**json.loads(
            Path(os.environ['TWIN_ARRIVAL_SHADOW']).read_text()))
        if len(_SHADOW_PROFILE.continuous) != 7:
            raise ValueError('Arrival shadow requires the seven arm joints')
    # S17: MoveItPy reads plan-request params from the 'ompl.plan_request_params.*'
    # config namespace. The builder does NOT populate it, so without this block
    # the six 'Parameter ompl.plan_request_params.* not found' warnings fire and
    # planning runs on built-in defaults. planner_id MUST match a config name that
    # now exists in twin_moveit_config/config/ompl_planning.yaml (added this
    # session): RRTConnectkConfigDefault, NOT bare 'RRTConnect' (which produced
    # 'Cannot find planning configuration for group panda_arm').
    d.setdefault("ompl", {})["plan_request_params"] = {
        "planning_pipeline": "ompl",
        **_REQUEST_OPTIONS,
        "max_velocity_scaling_factor": VEL_SCALE*_retime_margin(),
        "max_acceleration_scaling_factor": ACC_SCALE*_retime_margin(),
    }
    d["use_sim_time"] = True
    smoother = os.environ.get('TWIN_SMOOTHER', 'off')
    if smoother not in ('off', 'stomp'):
        raise ValueError(f'Unsupported TWIN_SMOOTHER: {smoother}; available: off, stomp')
    _EXPERIMENT_PIPELINE = 'ompl'
    if smoother == 'stomp':
        if check != 'sampled':
            raise ValueError('STOMP experiment requires TWIN_TRAJECTORY_CHECK=sampled')
        if _REQUEST_OPTIONS['planner_id'] != motion_audit.DEFAULT_PLANNER:
            raise ValueError('STOMP comparison requires the incumbent RRTConnect planner')
        import copy
        smooth = copy.deepcopy(d['ompl'])
        smooth['planning_plugins'] = ['ompl_interface/OMPLPlanner', 'stomp_moveit/StompPlanner']
        smooth['stomp_moveit'] = dict(num_timesteps=60, num_iterations=20,
            num_iterations_after_valid=0, num_rollouts=10, max_rollouts=10,
            exponentiated_cost_sensitivity=.8, control_cost_weight=.1, delta_t=.1)
        smooth['plan_request_params']['planning_pipeline'] = 'ompl_smooth'
        d['ompl_smooth'] = smooth
        d['planning_pipelines']['pipeline_names'].append('ompl_smooth')
        _EXPERIMENT_PIPELINE = 'ompl_smooth'
    return d


def make_params(robot):
    global _FALLBACK_PARAMS
    p = PlanRequestParameters(robot, _EXPERIMENT_PIPELINE)
    p.planning_pipeline = _EXPERIMENT_PIPELINE
    # Match the config name registered in ompl_planning.yaml (see build_config_dict).
    options = _REQUEST_OPTIONS or motion_audit.request_options({motion_audit.DEFAULT_PLANNER: {}})
    p.planner_id = options['planner_id']
    p.planning_time = options['planning_time']
    p.planning_attempts = options['planning_attempts']
    p.max_velocity_scaling_factor = VEL_SCALE*_retime_margin()
    p.max_acceleration_scaling_factor = ACC_SCALE*_retime_margin()
    _FALLBACK_PARAMS = PlanRequestParameters(robot, 'ompl') if _EXPERIMENT_PIPELINE != 'ompl' else None
    return p


# ---- SEAM 1: setup (construct the ONE MoveItPy context; call once per process) ----
def setup(node_name="move_to", prime_gripper=True):
    global _SHADOW_NODE, _SHADOW_EXECUTOR
    rclpy.init()
    logger = get_logger(node_name)
    params_file = create_params_file_from_dict(build_config_dict(logger), "/**")
    robot = MoveItPy(node_name=node_name, launch_params_filepaths=[params_file])
    arm = robot.get_planning_component(ARM_GROUP)
    params = make_params(robot)

    # Our OWN lightweight node, separate from MoveItPy's internal (already-spun)
    # node. This is the stable home for non-MoveIt ROS interfaces: the gripper
    # ActionClient now, and later the grasp-success publisher / perception subs
    # that pick/place will need. Spinning MoveItPy's own node here would double-
    # spin its current-state-monitor executor and deadlock — so we keep ours
    # isolated and drive it with a private SingleThreadedExecutor per call.
    twin = Node("twin_gripper")
    if _SHADOW_PROFILE is not None:
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        _SHADOW_NODE = Node('motion_arrival_shadow', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        _SHADOW_NODE.create_subscription(JointState, '/joint_states', _on_shadow_joint_states,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        _SHADOW_EXECUTOR = SingleThreadedExecutor()
        _SHADOW_EXECUTOR.add_node(_SHADOW_NODE)
    twin.create_subscription(JointState, "/joint_states", _on_joint_states, 10)
    # Action client hangs off the node so the free-function primitive reaches it
    # via the handle we return. Now a FollowJointTrajectory client (JTC over both
    # fingers), not GripperCommand -- see move_gripper for why.
    twin._gripper_client = ActionClient(twin, FollowJointTrajectory, GRIPPER_ACTION)

    # S20 PERCEPTION: create the live-pose cache on the twin node's executor.
    # The camera sub rides twin's spin (no new thread); resolve_object reads
    # the latest cube pose from it instead of the old hardcoded dict.
    global _POSE_CACHE
    _POSE_CACHE = _ObjectPoseCache(twin, logger)

    # A3: optionally publish the cell's bins into the planning scene so motion is
    # genuinely collision-checked against them. Opt-in via TWIN_CELL_COLLISION=1 so the
    # ground-truth-verified A1/A2 paths keep the exact planning behaviour they passed
    # under; a failure here is logged and ignored, never fatal.
    try:
        from twin_moveit_scripts.cell_scene import maybe_add_cell_collision_objects
        maybe_add_cell_collision_objects(robot, logger)
    except Exception as e:
        logger.warn(f"setup: cell collision objects skipped: {e!r}")

    logger.info("MoveItPy up; panda_arm ready. twin_gripper node up.")
    if not prime_gripper:
        # Continuous parcel startup owns controller readiness and overlaps its
        # verified gripper warm-up/open with the initial collision-checked move.
        return robot, arm, logger, params, twin
    time.sleep(2.0)  # let trajectory-execution action client connect to arm_controller

    # A4: PRIME THE GRIPPER -- burn the first finger command here.
    # Measured: the FIRST finger command in a process is reliably swallowed. The
    # controller accepts the goal and returns a terminal result, but the fingers never
    # leave their 0.02 spawn position. Retrying does NOT recover it (three attempts
    # 1.5 s apart all returned 0.02), yet the next command later in the process always
    # succeeds -- so it is first-command initialisation, not a timing window a retry
    # can win. A half-open hand then fouls the descend onto a 3 cm object: the arm
    # stalls ~29 mm high and shoves the object (one cylinder was knocked 0.3 m).
    # One throwaway stroke here costs ~1.5 s and makes the first REAL open the second
    # command. Non-fatal: a failure is logged and the verify+retry in move_gripper
    # remains as the safety net.
    try:
        _pexec = SingleThreadedExecutor()
        _pexec.add_node(twin)
        try:
            if twin._gripper_client.wait_for_server(timeout_sec=5.0):
                _ok, _w, _err = _send_finger_goal(
                    twin._gripper_client, _pexec, twin, GRIPPER_MAX, logger)
                logger.info(f"setup: gripper prime stroke -> ok={_ok} width={_w}")
            else:
                logger.warn("setup: gripper action server absent; prime skipped")
        finally:
            _pexec.remove_node(twin)
    except Exception as e:
        logger.warn(f"setup: gripper prime raised: {e!r}")

    return robot, arm, logger, params, twin


# ---- SEAM 3: teardown ----
def shutdown(twin=None):
    global _SHADOW_NODE, _SHADOW_EXECUTOR
    if _SHADOW_EXECUTOR is not None:
        _SHADOW_EXECUTOR.remove_node(_SHADOW_NODE)
        _SHADOW_EXECUTOR.shutdown(timeout_sec=0)
        _SHADOW_NODE.destroy_node()
        _SHADOW_NODE = _SHADOW_EXECUTOR = None
    if twin is not None:
        try:
            twin.destroy_node()
        except Exception:
            pass
    rclpy.shutdown()


def _read_ee_xyz(robot, logger):
    """Read the ACTUAL end-effector xyz from the robot's live current state.

    We report where the arm truly is, not the goal we asked for -- 'asked' and
    'actual' diverge exactly in the cases self-correction cares about (plan
    clamped, drift, partial execution). Returns None if the state can't be read,
    so a pose failure never masquerades as a motion failure.
    """
    try:
        state = robot.get_planning_component(ARM_GROUP).get_start_state()
        tf = state.get_global_link_transform(EE_LINK)
        # tf is a 4x4 homogeneous transform; translation is the last column.
        return (float(tf[0][3]), float(tf[1][3]), float(tf[2][3]))
    except Exception as e:
        logger.warn(f"could not read live EE pose: {e}")
        return None


# ---- /joint_states callback: caches ONLY panda_finger_joint1's live value.
# The broadcaster publishes all joints in one message; we index our one by name.
# Cache lives as a function attribute (single-process, single gripper) so both
# the callback and _read_finger_width reach it without threading it through args.
def _on_joint_states(msg):
    # S21: width = MEAN of BOTH finger joints, not finger 1 alone.
    # The two fingers are independent position-driven joints (no mimic) and do NOT
    # move symmetrically: after the first grasp/release finger 1 closes/opens at
    # ~4 mm/s while finger 2 keeps ~16 mm/s, and a held cube see-saws +/-4 mm
    # between them (q1 and q2 swap while q1+q2 stays at the cube width). Reading
    # finger 1 only therefore measured WHERE THE CUBE SAT in that see-saw, not the
    # grip: an off-centre grasp read as "stopped at 0.029", the hold stroke then
    # commanded BOTH fingers to 0.024 (opening finger 2 and dropping the cube), and
    # pick() reported "nothing grasped" over a cube that was in the hand. The mean
    # keeps the old units (per-finger metres, GRIPPER_MAX = 0.04), so every
    # threshold below is unchanged, and it is invariant to the see-saw.
    try:
        i1 = msg.name.index(FINGER_JOINTS[0])
        i2 = msg.name.index(FINGER_JOINTS[1])
    except ValueError:
        return  # this message didn't carry both finger joints; ignore
    _on_joint_states.latest = 0.5 * (float(msg.position[i1]) + float(msg.position[i2]))


_on_joint_states.latest = None
_on_joint_states.arm_sample = None


def _on_shadow_joint_states(msg):
    try:
        indices = [msg.name.index(n) for n in motion_audit.ARM_JOINTS]
        _on_joint_states.arm_sample = dict(
            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9,
            positions=[float(msg.position[i]) for i in indices],
            velocities=[float(msg.velocity[i]) for i in indices])
    except (ValueError, IndexError):
        _on_joint_states.arm_sample = None


def _read_finger_width(twin, logger, exec_=None):
    """Read the ACTUAL panda_finger_joint1 value live from /joint_states.

    Exact mirror of _read_ee_xyz: report where the finger truly is, not the width
    we asked for. Pumps callbacks briefly so a fresh message lands rather than
    trusting a possibly-stale cache. Returns None on failure so a read miss never
    masquerades as a grasp failure.

    If `exec_` is given (move_gripper's own executor, already holding `twin`), we
    reuse it -- a node cannot belong to two executors at once. If not, we make a
    throwaway one and clean it up.
    """
    owned = exec_ is None
    try:
        if owned:
            exec_ = SingleThreadedExecutor()
            exec_.add_node(twin)
        _on_joint_states.latest = None  # force a genuinely fresh sample
        deadline = time.time() + 1.0
        while time.time() < deadline:
            exec_.spin_once(timeout_sec=0.1)
            if _on_joint_states.latest is not None:
                break
        return _on_joint_states.latest
    except Exception as e:
        logger.warn(f"could not read live finger width: {e}")
        return None
    finally:
        if owned and exec_ is not None:
            exec_.remove_node(twin)


class _TenClock:
    """Fresh volatile /clock subscription for the ten-profile gripper deadline.

    rclpy.qos has no qos_profile_clock in Jazzy, so the profile is explicit. It
    is opened BEFORE the gripper goal is dispatched: an import or subscription
    failure must surface while nothing has been accepted by the controller.
    """

    def __init__(self, twin):
        from rosgraph_msgs.msg import Clock
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        self.twin = twin
        self.sample = None
        profile = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
        self.subscription = twin.create_subscription(Clock, '/clock', self._on_clock, profile)

    def _on_clock(self, message):
        self.sample = message.clock.sec + message.clock.nanosec/1e9

    def close(self):
        self.twin.destroy_subscription(self.subscription)


def _wait_ten_finger_result(exec_, twin, future, clock, move_time=None):
    from twin_moveit_scripts.gripper_deadline import SimulationDeadline

    # A new volatile subscription cannot reuse this node's stale cached /clock
    # after a long blocking MoveIt call. Start from the first fresh publication.
    clock.sample = None
    deadline = SimulationDeadline(time.monotonic(),
                                  (GRIPPER_MOVE_TIME if move_time is None else move_time)+2.)
    while not future.done():
        exec_.spin_once(timeout_sec=.05)
        if future.done():
            return None
        error = deadline.check(time.monotonic(), clock.sample)
        if error:
            return error
    return None


def _cancel_finger_goal(goal_handle, exec_):
    """Best-effort cancel of an accepted gripper goal; returns whether acknowledged."""
    try:
        cancellation = goal_handle.cancel_goal_async()
        _spin_until(exec_, cancellation, timeout=3.)
        ack = cancellation.result() if cancellation.done() else None
        return bool(ack is not None and ack.goals_canceling)
    except Exception:
        return False


def _send_finger_goal(client, exec_, twin, target, logger, settle=None, seconds=None):  # settle: inert (S19)
    """Send BOTH fingers to `target` via one FollowJointTrajectory point, await the
    action, let the fingers settle, and return the live finger width (or None).

    The shared plumbing for every finger motion. Returns (ok, final_width, err):
      ok False + err set  -> action failed (server/reject/timeout)
      ok True             -> motion ran; final_width is the settled live read
    """
    traj = JointTrajectory()
    traj.joint_names = list(FINGER_JOINTS)
    pt = JointTrajectoryPoint()
    pt.positions = [target, target]
    move_time = GRIPPER_MOVE_TIME if seconds is None else float(seconds)
    secs = int(move_time)
    pt.time_from_start = Duration(sec=secs,
                                  nanosec=int((move_time - secs) * 1e9))
    traj.points = [pt]
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj

    ten = _extended_profile()
    ten_clock = _TenClock(twin) if ten else None  # before dispatch: fail with nothing accepted
    try:
        send_future = client.send_goal_async(goal)
        _spin_until(exec_, send_future, timeout=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, None, "goal rejected by controller"

        result_future = goal_handle.get_result_async()
        if ten:
            try:
                error = _wait_ten_finger_result(exec_, twin, result_future, ten_clock, move_time)
            except BaseException:
                _cancel_finger_goal(goal_handle, exec_)  # never leave an accepted goal running
                raise
            if error:
                cancelled = _cancel_finger_goal(goal_handle, exec_)
                return False, None, f'{error}; cancellation acknowledged={cancelled}'
        else:
            _spin_until(exec_, result_future, timeout=15.0)
    finally:
        if ten_clock is not None:
            ten_clock.close()
    wrapped = result_future.result()
    if wrapped is None:
        return False, None, "no result returned before timeout"
    # A non-zero error_code here is EXPECTED when an object blocks the fingers
    # (goal not reachable) -- width is the source of truth, not error_code.

    # Settle: the fast prismatic joints may still be coasting when the action
    # returns. S19: wait until the width STOPS CHANGING rather than for a fixed
    # window. IMPORTANT: enforce a MINIMUM floor first -- _read_finger_width can
    # return two near-identical samples from inside one /joint_states interval
    # while the finger is still moving, which falsely trips the "settled" test in
    # ~0.3s. The floor guarantees we watch the width across several real messages
    # before believing it has stopped. Also pace each sample by GRIPPER_SETTLE_DT
    # so prev/w come from distinct messages.
    t0 = time.time()
    first = _read_finger_width(twin, logger, exec_=exec_)
    prev, stable_since, w = first, None, first
    floor = t0 + GRIPPER_SETTLE_MIN
    deadline = t0 + GRIPPER_SETTLE_MAX
    # S20: `settled` was derived as (stable_since is not None) AFTER the loop, so
    # exiting on the DEADLINE with stable_since freshly set -- i.e. the fingers had
    # gone quiet for less than GRIPPER_SETTLE_HOLD -- still printed "(settled)".
    # That is a false pass on exactly the signal §7 tells the next session to read
    # when judging the slow loaded-open against the 6 s cap. Now settled is set
    # ONLY where the hold duration is actually satisfied, and the cap case is
    # reported in two distinguishable flavours.
    settled = False
    while time.time() < deadline:
        s = time.time()
        while time.time() - s < GRIPPER_SETTLE_DT:
            exec_.spin_once(timeout_sec=0.02)
        w = _read_finger_width(twin, logger, exec_=exec_)
        if w is None:
            continue
        moving = prev is None or abs(w - prev) >= GRIPPER_SETTLE_TOL
        if moving or time.time() < floor:
            stable_since = None          # not allowed to settle before the floor
        elif stable_since is None:
            stable_since = time.time()
        elif time.time() - stable_since >= GRIPPER_SETTLE_HOLD:
            settled = True
            break
        prev = w

    elapsed = time.time() - t0
    if settled:
        verdict = "(settled)"
    elif stable_since is not None:
        # Went quiet, but the cap arrived before it had been quiet long enough.
        # Probably a real stop we clipped -- and a direct signal that SETTLE_MAX
        # is too tight for this stroke. Distinct from never having stopped.
        verdict = (f"(QUIET {time.time() - stable_since:.2f}s BUT CAP HIT at "
                   f"{GRIPPER_SETTLE_MAX}s -- raise SETTLE_MAX)")
    else:
        verdict = f"(STILL MOVING at {GRIPPER_SETTLE_MAX}s cap)"
    logger.info(f"_send_finger_goal: target={target:.4f} "
                f"first={first if first is None else round(first, 4)} "
                f"final={w if w is None else round(w, 4)} "
                f"in {elapsed:.2f}s {verdict}")
    if not settled:
        logger.warn(f"_send_finger_goal: fingers had not verifiably stopped after "
                    f"{GRIPPER_SETTLE_MAX}s; width reading may be mid-stroke")
    return True, w, None


# ---- Grasp record (S20) ----
# The bug this fixes: place() confirmed release RELATIVELY (opened >=
# GRASP_RELEASE_MARGIN past the grasp width) but tested "are we holding?"
# ABSOLUTELY (width > GRIPPER_MAX - 0.003). Those disagree in the middle: a hand
# that opened 0.0123 -> 0.0203 is physically clear of a 30 mm cube and was
# reported RELEASED, yet the next place() would call the same 0.0203 "holding"
# and run a whole approach/descend/open cycle on an empty hand. Two different
# questions were being answered by two different rulers.
#
# Fix: keep the ONE fact that settles it -- where the object seated the fingers --
# and judge both from it with the SAME constant (GRASP_RELEASE_MARGIN).
# Tri-state, because "no information" and "known empty" are not the same thing:
#   known=False            -> nothing observed this process; fall back to the
#                             absolute test and SAY SO (the CLI `place` case,
#                             where pick ran in a different process)
#   known=True, width=None -> observed a confirmed release; definitely not holding
#   known=True, width=w    -> holding, seated at w
_GRASP_STATE = {"known": False, "width": None}


def _record_grasp(width, logger):
    _GRASP_STATE.update(known=True, width=width)
    logger.info(f"grasp record: HOLDING at width={width:.4f}")


def _clear_grasp(logger, why):
    _GRASP_STATE.update(known=True, width=None)
    logger.info(f"grasp record: cleared ({why})")


def move_gripper(robot, arm, logger, params, twin, width,
                 max_effort=GRIPPER_DEFAULT_EFFORT, seconds=None) -> GripResult:
    """Move BOTH fingers to `width` via FollowJointTrajectory, await, judge grasp.

    `width` is the target value of each finger joint (0.0 closed .. 0.04 open),
    clamped to the URDF limit. Both fingers are commanded to the same value so the
    hand closes symmetrically (fixes the DART-dropped-mimic asymmetry). ACTION, not
    a MoveIt plan.

    TWO-STAGE CLOSE (S17). A single command to 0.0 against a rigid object the
    fingers can't reach leaves a permanent position error, so the JTC pushes inward
    forever and the held object oscillates. So on a CLOSING command we:
      1. DETECT stroke: drive toward `width` until the object stops the fingers;
         read where they actually halted (= object half-open width w).
      2. If something was grasped, HOLD command: re-drive to (w - GRASP_SQUEEZE),
         a reachable-but-for-a-margin target -> firm, SETTLED hold, no oscillation.
    An opening command (or a close that grasps nothing) uses a single stroke.

    GRASP DETECTION replaces the old controller `stalled` flag (which gave false
    HELDs). We infer the grasp from the detect stroke: fingers halting WIDER than
    commanded by > GRASP_GAP_MIN means an object blocked them. `stalled` in the
    result is this DERIVED signal. Object-size-agnostic.

    `max_effort` is kept for signature compatibility but is inert (JTC position
    trajectories carry no effort field).
    """
    c = max(GRIPPER_MIN, min(GRIPPER_MAX, float(width)))
    if c != float(width):
        logger.warn(f"clamped gripper width: {width} -> {c} "
                    f"(limit {GRIPPER_MIN}..{GRIPPER_MAX})")
    closing = c < GRIPPER_MAX  # a move toward closed; only then can a grasp occur
    logger.info(f"=== move_gripper(width={c}, closing={closing}) ===")

    client = twin._gripper_client
    exec_ = SingleThreadedExecutor()
    exec_.add_node(twin)
    try:
        if not client.wait_for_server(timeout_sec=5.0):
            return GripResult(ok=False, stage="error",
                              error=f"{GRIPPER_ACTION} action server not available")

        # --- Stage 1: detect stroke ---
        ok, final_width, err = _send_finger_goal(client, exec_, twin, c, logger, seconds=seconds)
        if not ok:
            stage = "rejected" if "rejected" in (err or "") else "error"
            return GripResult(ok=False, stage=stage, error=err)

        # --- Stage 1b (A3): VERIFY AN OPEN ACTUALLY OPENED ---
        # See GRIPPER_OPEN_TOL. An opening stroke that settles well short of its target
        # did not happen -- almost always the first command after bringup being dropped
        # by a not-yet-ready controller. Retry rather than hand a half-open hand to the
        # descend. A CLOSE is exempt: stopping short is how a grasp is detected.
        attempt = 1
        while (not closing) and ok and final_width is not None \
                and (c - final_width) > GRIPPER_OPEN_TOL \
                and attempt < GRIPPER_OPEN_ATTEMPTS:
            attempt += 1
            logger.warn(f"move_gripper: OPEN did not take -- fingers at "
                        f"{final_width:.4f} vs commanded {c:.4f} "
                        f"(short by {c - final_width:.4f} > {GRIPPER_OPEN_TOL}); "
                        f"waiting {GRIPPER_OPEN_RETRY_WAIT}s then re-sending "
                        f"(attempt {attempt}/{GRIPPER_OPEN_ATTEMPTS})")
            # Pump the executor while waiting so joint states keep flowing; a bare
            # sleep would race the same unready controller the first send hit.
            t_end = time.time() + GRIPPER_OPEN_RETRY_WAIT
            while time.time() < t_end:
                exec_.spin_once(timeout_sec=0.05)
            ok, final_width, err = _send_finger_goal(client, exec_, twin, c, logger, seconds=seconds)
        if (not closing) and final_width is not None \
                and (c - final_width) > GRIPPER_OPEN_TOL:
            logger.error(f"move_gripper: open STILL short after "
                         f"{GRIPPER_OPEN_ATTEMPTS} attempts "
                         f"(fingers {final_width:.4f}, commanded {c:.4f}) -- a "
                         f"half-open hand will foul a descend onto a 3 cm object")
            return GripResult(ok=False, stage="error", final_width=final_width,
                              error=f"open did not reach: {final_width:.4f} < {c:.4f}")

        # Judge the grasp from where the fingers stopped.
        if final_width is None:
            held = False
            gap = None
        else:
            gap = final_width - c
            held = closing and (gap > GRASP_GAP_MIN)

        # --- Stage 2: hold command (only if we actually grasped something) ---
        if held and final_width is not None:
            hold_target = max(GRIPPER_MIN, final_width - GRASP_SQUEEZE)
            logger.info(f"move_gripper: grasp at {final_width:.4f}; "
                        f"holding at {hold_target:.4f} (squeeze {GRASP_SQUEEZE}) "
                        f"to settle the grip")
            ok2, hold_width, err2 = _send_finger_goal(client, exec_, twin, hold_target, logger)
            if ok2 and hold_width is not None:
                # S19: VERIFY THE HOLD. A real object blocks the hold stroke too,
                # so the fingers settle WIDER than hold_target (by ~GRASP_SQUEEZE).
                # If they land ON the target, nothing is being compressed: the
                # detect stroke's halt was a transient, not an object, and the
                # "grip" is air. This is the check that would have caught the S19
                # run-1 failure -- detect 0.0204, hold_target 0.0154, settled at
                # 0.0154 exactly, i.e. 0.4 mm clear of a 15 mm half-width cube.
                # S20: was GRASP_GAP_MIN (shared with the detect floor and place's
                # empty test); now its own GRASP_HOLD_MIN. Log the measured
                # compression so a near-threshold hold is visible BEFORE it flips.
                compression = hold_width - hold_target
                logger.info(f"move_gripper: hold compression {compression:.4f} "
                            f"(floor {GRASP_HOLD_MIN}, squeeze {GRASP_SQUEEZE})")
                if compression <= GRASP_HOLD_MIN:
                    logger.warn(f"move_gripper: hold reached its target exactly "
                                f"({hold_width:.4f} vs {hold_target:.4f}); nothing "
                                f"is being squeezed -> detect stall was spurious")
                    held = False
                final_width = hold_width  # report where it actually settled

        # S20: move_gripper is the ONE place that knows whether an object ended up
        # between the fingers, so it is the only writer of the grasp record.
        # A close that grasped nothing clears it just as an open does -- either way
        # the hand is empty.
        if held and final_width is not None:
            _record_grasp(final_width, logger)
        else:
            _clear_grasp(logger, "gripper move ended with nothing held")

        stage = "stalled" if held else "reached"  # keep 'stalled' label == held
        gr = GripResult(ok=True, stage=stage, stalled=held, final_width=final_width)
        gapstr = f"{gap:.4f}" if gap is not None else "?"
        logger.info(f"move_gripper -> {gr} (gap={gapstr}, floor={GRASP_GAP_MIN})")
        return gr
    except Exception as e:
        logger.error(f"move_gripper raised: {e}")
        return GripResult(ok=False, stage="error", error=str(e),
                          final_width=_read_finger_width(twin, logger, exec_=exec_))
    finally:
        exec_.remove_node(twin)


def _spin_until(exec_, future, timeout):
    """Spin the given executor until `future` completes or `timeout` elapses."""
    deadline = time.time() + timeout
    while not future.done() and time.time() < deadline:
        exec_.spin_once(timeout_sec=0.1)


# S21: how far (m) the live EE may sit from the plan's last waypoint and still count
# as "arrived". Well above tracking error (~2-4 mm), well below any real move.
ARRIVAL_TOL = 0.02
EXEC_ATTEMPTS = 2


def _planned_end_xyz(trajectory, logger):
    """EE xyz at the trajectory's last waypoint, or None if it can't be read."""
    try:
        tf = trajectory[len(trajectory) - 1].get_global_link_transform(EE_LINK)
        return (float(tf[0][3]), float(tf[1][3]), float(tf[2][3]))
    except Exception as e:
        logger.warn(f"could not read planned end pose: {e}")
        return None


def _settle_pose(robot, arm, logger):
    """Wait until the live EE pose stops changing (or SETTLE_CAP elapses), then
    return it. Replaces the old blind fixed sleep after every segment:
    robot.execute() already blocks until the trajectory finishes, so we only need
    to confirm the arm has stopped ringing before sampling the arrival pose.
    Refreshes the planning component's start state each sample so _read_ee_xyz
    reflects the live current state rather than a stale snapshot.
    """
    deadline = time.time() + SETTLE_CAP
    prev = None
    still_since = None
    last = None
    while True:
        _shadow_arrival_tick()
        try:
            arm.set_start_state_to_current_state()
        except Exception:
            pass
        last = _read_ee_xyz(robot, logger)
        now = time.time()
        if prev is not None and last is not None:
            moved = sum((a - b) ** 2 for a, b in zip(last, prev)) ** 0.5
            if moved < SETTLE_TOL:
                if still_since is None:
                    still_since = now
                elif now - still_since >= SETTLE_HOLD:
                    _shadow_arrival_end('existing_pose_dwell')
                    return last
            else:
                still_since = None
        prev = last
        if now >= deadline:
            _shadow_arrival_end('existing_settle_cap')
            return last
        time.sleep(SETTLE_DT)


def _shadow_arrival_tick():
    """Observe alongside the old settle predicate; never extend or shorten its wait."""
    if _SHADOW_COMMAND is None:
        return
    command = _SHADOW_COMMAND
    try:
        if _SHADOW_EXECUTOR is not None:
            for _ in range(4):
                _SHADOW_EXECUTOR.spin_once(timeout_sec=0)
        now = _SHADOW_NODE.get_clock().now().nanoseconds/1e9
        command['now'] = now
        gate = command['gate']
        if gate.command_id is None:
            gate.start(command['command_id'], command['target'], now)
        if not gate.terminal:
            sample = _on_joint_states.arm_sample
            command['status'] = gate.update(now=now,
                stamp=None if sample is None else sample['stamp'],
                positions=None if sample is None else sample['positions'],
                velocities=None if sample is None else sample['velocities'],
                result_command_id=command['command_id'],
                controller_succeeded=command['controller_succeeded'])
    except Exception as exc:
        command['status'] = 'SHADOW_NOT_CHECKED'
        command['error'] = str(exc)


def _shadow_arrival_end(existing_reason):
    global _SHADOW_COMMAND
    if _SHADOW_COMMAND is not None:
        command = _SHADOW_COMMAND
        motion_audit.record('arrival_shadow', command_id=command['command_id'],
            status=command.get('status', 'SHADOW_NOT_CHECKED'),
            error=command.get('error'), existing_reason=existing_reason,
            sample_clock_domain='node_ros_time', ros_stamp_ns=int(command.get('now', 0)*1e9),
            execution_authorized=False)
        _SHADOW_COMMAND = None


def _timed_plan(arm, params, kind):
    started = time.monotonic()
    result = None
    effective_pipeline = params.planning_pipeline
    try:
        result = arm.plan(single_plan_parameters=params)
        if not result and params.planning_pipeline == 'ompl_smooth':
            _check_motion()
            motion_audit.record('fallback', reason='SMOOTHER_PLAN_FAILED', kind=kind,
                                pipeline_id='ompl', planner_id=_FALLBACK_PARAMS.planner_id)
            effective_pipeline = 'ompl'
            result = arm.plan(single_plan_parameters=_FALLBACK_PARAMS)
        return result
    finally:
        motion_audit.record('plan_finished', kind=kind, planner_id=params.planner_id,
            pipeline_id=effective_pipeline, requested_pipeline_id=params.planning_pipeline,
            planning_wall_s=time.monotonic()-started, success=bool(result),
            planning_time=params.planning_time, planning_attempts=params.planning_attempts)


def _audit_candidate(robot, trajectory, params, kind):
    """Optional checks on FINAL retimed motion, with current attachments retained."""
    command_id = motion_audit.identifier()
    checking = os.environ.get('TWIN_TRAJECTORY_CHECK', 'off') == 'sampled'
    if not checking and not os.environ.get('TWIN_MOTION_METRICS'):
        return command_id
    import copy
    from rclpy.serialization import serialize_message
    import hashlib
    data = motion_audit.trajectory_data(trajectory.get_robot_trajectory_msg())
    verdict = None
    ee = []
    collision_samples = 0
    with robot.get_planning_scene_monitor().read_only() as scene:
        scene_hash = hashlib.sha256(serialize_message(scene.planning_scene_message)).hexdigest()
        state = copy.deepcopy(scene.current_state)
        measured_start = list(state.get_joint_group_positions(ARM_GROUP))
        if checking:
            verdict = motion_audit.validate(data, _MOTION_LIMITS,
                stationary_start=measured_start)
        for point in data['points']:
            state.set_joint_group_positions(ARM_GROUP, point['positions'])
            state.update(force=True)
            tf = state.get_global_link_transform(EE_LINK)
            ee.append([float(tf[i][3]) for i in range(3)])
        if checking and verdict['sampled_checks_passed']:
            for positions in motion_audit.interpolated_positions(data):
                collision_samples += 1
                state.set_joint_group_positions(ARM_GROUP, positions)
                state.update(force=True)
                if scene.is_state_colliding(robot_state=state, joint_model_group_name=''):
                    verdict['errors'].append('SAMPLED_COLLISION')
                    break
            verdict['sampled_checks_passed'] = not verdict['errors']
            verdict['collision_samples'] = collision_samples
    motion_audit.record('candidate', command_id=command_id, kind=kind,
        planner_id=params.planner_id, timer='TOTG', profile_hash=_MOTION_PROFILE_HASH,
        scene_hash=scene_hash, trajectory_hash=motion_audit.digest(data), trajectory=data,
        measured_start=measured_start, requested_pipeline_id=params.planning_pipeline,
        limits=_MOTION_LIMITS, validation=verdict, metrics=motion_audit.metrics(data, ee))
    if checking and not verdict['sampled_checks_passed']:
        raise ValueError('VALIDATION_FAILED: ' + ','.join(verdict['errors'][:8]))
    return command_id


def _execute_audited(robot, trajectory, command_id):
    global _SHADOW_COMMAND
    motion_audit.record('execution_started', command_id=command_id)
    started = time.monotonic()
    try:
        result = robot.execute(trajectory, controllers=[])
        if _SHADOW_PROFILE is not None:
            from twin_moveit_scripts.arrival_gate import ArrivalGate
            target = trajectory.get_robot_trajectory_msg().joint_trajectory.points[-1].positions
            _SHADOW_COMMAND = dict(command_id=command_id, target=list(target),
                gate=ArrivalGate(_SHADOW_PROFILE), controller_succeeded=bool(result))
        motion_audit.record('execution_finished', command_id=command_id,
            wall_s=time.monotonic()-started,
            return_value=str(getattr(result, 'status', result)))
        return result
    except Exception as exc:
        motion_audit.record('execution_failed', command_id=command_id, error=str(exc))
        raise


def _plan_execute(robot, arm, logger, params, sleep_time=3.0):  # sleep_time: inert (Phase 1)
    """Plan, execute, VERIFY ARRIVAL, then read back the live pose. Returns a MoveResult.

    S21: execution's outcome used to be ignored, so ok=True meant only "nothing
    raised". MoveIt's execution watchdog can cancel a trajectory ~50 ms after it
    starts (seen once: TIMED_OUT against a 4.8 s bound), the arm stays put, and
    the caller carries on from the wrong place -- pick() then looked at the cube
    with the hand still over it. Arrival is now judged by where the arm IS against
    where the plan ended, and a missed move is replanned once from the live state.
    """
    for attempt in range(1, EXEC_ATTEMPTS + 1):
        _check_motion()
        arm.set_start_state_to_current_state()
        try:
            result = _timed_plan(arm, params, 'single')
        except Exception as e:
            logger.error(f"Planning raised: {e}")
            return MoveResult(ok=False, stage="plan_failed", error=str(e))

        if not result:
            logger.error("Planning FAILED.")
            return MoveResult(ok=False, stage="plan_failed",
                              error="planner returned no trajectory")

        logger.info("Planning succeeded, executing...")
        _check_motion()
        try:
            command_id = _audit_candidate(robot, result.trajectory, params, 'single')
            _execute_audited(robot, result.trajectory, command_id)
            _check_motion()
        except Exception as e:
            logger.error(f"Execution raised: {e}")
            return MoveResult(ok=False, stage="exec_error",
                              error=str(e), final_pose=_settle_pose(robot, arm, logger))

        xyz = _settle_pose(robot, arm, logger)  # confirm stillness, then sample the pose
        end = _planned_end_xyz(result.trajectory, logger)
        if xyz is None or end is None:
            logger.warn("arrival not verifiable (pose unreadable); assuming executed")
            return MoveResult(ok=True, stage="executed", final_pose=xyz)
        miss = sum((a - b) ** 2 for a, b in zip(xyz, end)) ** 0.5
        if miss <= ARRIVAL_TOL:
            return MoveResult(ok=True, stage="executed", final_pose=xyz)
        logger.warn(f"execution did not arrive: {miss:.3f} m from planned end "
                    f"(attempt {attempt}/{EXEC_ATTEMPTS})")

    return MoveResult(ok=False, stage="exec_incomplete", final_pose=xyz,
                      error=f"arm stopped {miss:.3f} m from the planned end")


# ---- SEAM 2: primitives (take the live handles; the LLM node imports these) ----
def move_to_named(robot, arm, logger, params, name) -> MoveResult:
    logger.info(f"=== move_to_named('{name}') ===")
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name=name)
    res = _plan_execute(robot, arm, logger, params)
    logger.info(f"move_to_named('{name}') -> {res}")
    return res


def move_to_pose(robot, arm, logger, params, x, y, z, quat=None) -> MoveResult:
    """Move panda_link8 (EE_LINK) to a Cartesian point.

    `quat` is an optional (qx, qy, qz, qw) goal orientation for EE_LINK. When
    omitted it defaults to identity (w=1) -- the EXACT prior behavior, so the CLI
    and llm_move.py callers are unchanged. pick() passes a top-down grasp
    orientation here: position-only planning lets the planner choose any hand yaw,
    which is fine for reaching a point but unreliable for grasping (the box faces
    must line up with the fingers' closing line). See TOPDOWN_GRASP_QUAT.
    """
    logger.info(f"=== move_to_pose(x={x}, y={y}, z={z}, quat={quat}) ===")
    pose = PoseStamped()
    pose.header.frame_id = PLANNING_FRAME
    if quat is None:
        pose.pose.orientation.w = 1.0
    else:
        qx, qy, qz, qw = quat
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=pose, pose_link=EE_LINK)
    res = _plan_execute(robot, arm, logger, params)
    logger.info(f"move_to_pose -> {res}")
    return res


# ---- Deterministic grasp seed (S17) ----
# RRTConnect is randomized, and a 7-DOF arm has a null-space: many joint configs
# satisfy the same panda_link8 pose+quat. Left free, the planner resolves the
# redundant wrist (panda_joint7) differently each run, so the HAND (which sits at
# -45 deg off link8) meets the box at a different yaw -> grasp width varied wildly
# run to run (0.009 / 0.020 / 0.040) even after the planner-config fix. The fix:
# for the grasp poses, IK the top-down pose with a FIXED wrist seed so the hand
# yaw is pinned, then plan to that JOINT config. This is how industrial pick is
# taught -- grasp poses as joint targets, not Cartesian goals, for bit-repeatable
# approach. RRTConnect still plans the PATH; only the grasp POSTURE is pinned.
# panda_joint7 seed: 0.785398 (=+pi/4) cancels the hand's -pi/4 so the closing
# line is square to an axis-aligned box. Other joints seeded to the 'ready' basin.
GRASP_SEED_JOINTS = {
    "panda_joint1": 0.0,
    "panda_joint2": -0.785,
    "panda_joint3": 0.0,
    "panda_joint4": -2.356,
    "panda_joint5": 0.0,
    "panda_joint6": 1.571,
    "panda_joint7": 0.785398,   # pins hand yaw square to the box
}


def move_to_grasp_pose(robot, arm, logger, params, x, y, z, quat) -> MoveResult:
    """Like move_to_pose, but DETERMINISTIC: resolve the top-down pose to a fixed
    joint configuration (seeded IK, wrist pinned) and plan to that joint target,
    so the hand meets the object the same way every run. Falls back to the
    Cartesian move_to_pose if IK can't be solved (never worse than before).
    """
    logger.info(f"=== move_to_grasp_pose(x={x}, y={y}, z={z}) [seeded IK] ===")
    pose = PoseStamped()
    pose.header.frame_id = PLANNING_FRAME
    qx, qy, qz, qw = quat
    pose.pose.orientation.x = float(qx)
    pose.pose.orientation.y = float(qy)
    pose.pose.orientation.z = float(qz)
    pose.pose.orientation.w = float(qw)
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    try:
        # Build a RobotState from the model, seed it at the fixed grasp basin,
        # then IK toward the pose. Seeding biases set_from_ik's solver so the
        # redundant wrist resolves to the same value every run (deterministic).
        if _extended_profile():
            rs = _ten_profile_goal(robot,pose,logger)
            if rs is None:
                logger.warn('Ten-profile IK candidates rejected; trying collision-aware pose planning')
                return move_to_pose(robot,arm,logger,params,x,y,z,quat=quat)
            arm.set_start_state_to_current_state()
            arm.set_goal_state(robot_state=rs)
            return _plan_execute(robot,arm,logger,params)
        rs = RobotState(robot.get_robot_model())
        rs.set_joint_group_positions(ARM_GROUP,
            [GRASP_SEED_JOINTS[f"panda_joint{i}"] for i in range(1, 8)])
        rs.update(force=True)
        ok = rs.set_from_ik(joint_model_group_name=ARM_GROUP,
                            geometry_pose=pose.pose, tip_name=EE_LINK, timeout=1.0)
        if not ok:
            logger.warn("grasp IK failed; falling back to Cartesian pose goal")
            return move_to_pose(robot, arm, logger, params, x, y, z, quat=quat)
        rs.update(force=True)
        arm.set_start_state_to_current_state()
        arm.set_goal_state(robot_state=rs)
        res = _plan_execute(robot, arm, logger, params)
        logger.info(f"move_to_grasp_pose -> {res}")
        return res
    except Exception as e:
        logger.warn(f"seeded-IK grasp path errored ({e}); using Cartesian fallback")
        return move_to_pose(robot, arm, logger, params, x, y, z, quat=quat)


# ---- Phase 3: blended multi-waypoint motion ------------------------------------
# moveit_py plans each goal as its own point-to-point trajectory that starts and
# ends at ZERO velocity, so a pick that goes approach -> descend fully stops at the
# pre-grasp corner. To make the motion continuous ("natural, fewer stops") we plan
# each leg separately (each still collision-checked), CONCATENATE the paths, and
# re-time the whole thing with TOTG. TOTG only zeroes velocity at the two ENDS of
# the merged path, so the arm flows through the interior waypoints -- it slows at a
# sharp corner but does not stop. TOTG may change geometry within its path tolerance;
# TWIN_TRAJECTORY_CHECK=sampled checks the final output for experiments. Any failure
# returns None and the caller falls back to the original per-segment execution.
def _ten_profile_goal(robot,pose,logger):
    """Reject colliding IK branches before OMPL, retaining payload and hand state."""
    with robot.get_planning_scene_monitor().read_only() as scene:
        reference=copy.deepcopy(scene.current_state)
    fixed=[GRASP_SEED_JOINTS[n] for n in motion_audit.ARM_JOINTS]
    current=list(reference.get_joint_group_positions(ARM_GROUP))
    bearing=fixed.copy();bearing[0]=math.atan2(pose.pose.position.y,pose.pose.position.x)
    for number,seed in enumerate((fixed,current,bearing)):
        candidate=copy.deepcopy(reference)
        candidate.set_joint_group_positions(ARM_GROUP,seed)
        candidate.update(force=True)
        if not candidate.set_from_ik(joint_model_group_name=ARM_GROUP,
                geometry_pose=pose.pose,tip_name=EE_LINK,timeout=.2):continue
        candidate.update(force=True)
        q=list(candidate.get_joint_group_positions(ARM_GROUP))
        if _MOTION_LIMITS and not all(_MOTION_LIMITS[n]['position_min']<=v<=_MOTION_LIMITS[n]['position_max']
                                     for n,v in zip(motion_audit.ARM_JOINTS,q)):continue
        with robot.get_planning_scene_monitor().read_only() as scene:
            collision=scene.is_state_colliding(robot_state=candidate,joint_model_group_name='')
        motion_audit.record('ik_candidate',seed=number,accepted=not collision,
                           target=[pose.pose.position.x,pose.pose.position.y,pose.pose.position.z])
        if not collision:
            if number:logger.info(f'Ten-profile IK: selected collision-free seed {number}')
            return candidate
    return None


def _grasp_goal_state(robot, x, y, z, quat, logger):
    """Resolve a top-down (x,y,z,quat) pose to a seeded-IK goal RobotState (wrist
    pinned, same determinism as move_to_grasp_pose). Returns None if IK fails."""
    try:
        rs = RobotState(robot.get_robot_model())
        rs.set_joint_group_positions(ARM_GROUP,
            [GRASP_SEED_JOINTS[f"panda_joint{i}"] for i in range(1, 8)])
        rs.update(force=True)
        pose = PoseStamped()
        pose.header.frame_id = PLANNING_FRAME
        qx, qy, qz, qw = quat
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        if _extended_profile():
            return _ten_profile_goal(robot,pose,logger)
        if not rs.set_from_ik(joint_model_group_name=ARM_GROUP,
                              geometry_pose=pose.pose, tip_name=EE_LINK, timeout=1.0):
            return None
        rs.update(force=True)
        return rs
    except Exception as e:
        logger.warn(f"blend: grasp goal IK errored ({e})")
        return None


def _merge_trajectory_msgs(msgs):
    """Concatenate RobotTrajectory messages into one, dropping each segment's first
    point (it duplicates the previous segment's last) and offsetting time. TOTG
    re-times afterwards, so the input times only need to be monotonic."""
    import copy
    base = copy.deepcopy(msgs[0])
    pts = base.joint_trajectory.points
    def tsec(d):
        return d.sec + d.nanosec * 1e-9
    for m in msgs[1:]:
        seg = m.joint_trajectory.points
        if not seg:
            continue
        offset = tsec(pts[-1].time_from_start) if pts else 0.0
        for k, p in enumerate(seg):
            if k == 0:
                continue  # same config as the previous segment's last point
            q = copy.deepcopy(p)
            t = offset + tsec(p.time_from_start)
            q.time_from_start.sec = int(t)
            q.time_from_start.nanosec = int(round((t - int(t)) * 1e9))
            pts.append(q)
    return base


def _plan_chain_execute(robot, arm, logger, params, goal_states):
    """Plan a sequence of goal RobotStates, concatenate + re-time (TOTG) into ONE
    continuous motion, execute it, and verify arrival at the final goal. Returns a
    MoveResult on success, or None if any step fails (caller falls back)."""
    try:
        _check_motion()
        msgs = []
        container = None   # reuse the first planned trajectory so the merged one
                           # keeps a valid joint-model-group (TOTG requires it)
        start = None
        for goal in goal_states:
            if start is None:
                arm.set_start_state_to_current_state()
            else:
                arm.set_start_state(robot_state=start)
            arm.set_goal_state(robot_state=goal)
            result = _timed_plan(arm, params, 'chain_segment')
            if not result:
                logger.warn("blend: a chain segment failed to plan; falling back")
                return None
            if container is None:
                container = result.trajectory
            msgs.append(result.trajectory.get_robot_trajectory_msg())
            start = goal  # the planned segment ends at this goal configuration
        merged_msg = _merge_trajectory_msgs(msgs)
        arm.set_start_state_to_current_state()
        ref = arm.get_start_state()
        container.set_robot_trajectory_msg(ref, merged_msg)
        if not container.apply_totg_time_parameterization(VEL_SCALE*_retime_margin(), ACC_SCALE*_retime_margin()):
            logger.warn("blend: TOTG retiming failed; falling back")
            return None
        logger.info(f"blend: executing merged {len(goal_states)}-waypoint motion")
        _check_motion()
        command_id = _audit_candidate(robot, container, params, 'chain_totg')
        _execute_audited(robot, container, command_id)
        _check_motion()
        xyz = _settle_pose(robot, arm, logger)
        try:
            tf = goal_states[-1].get_global_link_transform(EE_LINK)
            end = (float(tf[0][3]), float(tf[1][3]), float(tf[2][3]))
        except Exception:
            end = None
        if xyz is not None and end is not None:
            miss = sum((a - b) ** 2 for a, b in zip(xyz, end)) ** 0.5
            if miss > ARRIVAL_TOL:
                logger.warn(f"blend: arrived {miss:.3f} m off planned end; falling back")
                return None
        return MoveResult(ok=True, stage="executed", final_pose=xyz)
    except Exception as e:
        logger.warn(f"blend: chain execution errored ({e}); falling back")
        return None


# ---- Perception seam (Session 17) — HARDCODED for now ----
# pick() needs object_id -> resting xyz (panda_link0/world frame; the two are
# identity here, confirmed S16 §3). Real perception (spawn objects, detect pose,
# reference by name) is a later roadmap brick. Until then this returns ONE known
# pose so pick exercises the real geometry (top-down orientation + fingertip
# offset) end-to-end.
#
# TODO(perception): replace the body with a real lookup (a /detected_objects
# subscription, a per-object TF frame, or an object_id->PoseStamped service).
# Only THIS function changes; pick() stays as-is.
#
# The placeholder is a spot dead-ahead of the base, inside the S16 §3b reachable
# box, with the test cube resting on the ground:
#   x=0.45 mid-reach, y=0.0 centered, z=0.015 = half of the 3 cm cube (resting
#   center). NOTE: the spawn command below uses -z 0.03 so the cube drops and
#   settles; the TABLE ENTRY must hold the settled center (0.015), not the -z.
# Spawn the cube there (NO -Y now -- the arm owns yaw alignment):
#   ros2 run ros_gz_sim create -world empty -name b \
#     -file ~/ros_test_ws/grasp_box.sdf -x 0.45 -y 0.0 -z 0.03
# S19 FIX: these z values are the object's RESTING CENTER, not the spawn z.
# grasp_box.sdf is a 0.03 cube, so a cube sitting on the ground plane has its
# center at 0.015. We were storing 0.030 (the -z we spawn at, from which the
# cube falls 15 mm and settles). That put z_grasp = 0.03 + FINGER_REACH_Z
# = 0.123, i.e. the pad centers exactly on the cube's TOP FACE -- the fingers
# closed on the top edge instead of around the body, the detect stroke halted
# ~5 mm too wide, and GRASP_SQUEEZE then held OUTSIDE the cube. Hence
# "reaches and closes fine but can't lift it".
# S20 PERCEPTION SEAM: 'box' now comes from LIVE perception (overhead camera),
# not a hardcoded spawn point. This is what unblocks repeat cycles -- after the
# cube is placed, the camera sees it at its NEW spot, so a second pick drives to
# where it ACTUALLY is. Place TARGETS ('table') stay fixed: a drop location is
# chosen, not perceived, so there is nothing to detect there.
_DETECT_TOPIC = "/detected_objects/box"
_DETECT_TIMEOUT = 3.0        # retained for reference; freshness is now handled by
                             # clear-and-wait in get_fresh(), not a stamp comparison

# The only survivor of the old _KNOWN_OBJECTS dict: place targets (fixed coords).
_KNOWN_TARGETS = {
    "table": (0.35, 0.2, 0.015),     # release at the cube's resting-center height
}


# ---- S22: auto-chosen, varying place destination (stateless ring-tour) ----
# The point of the repeat-without-respawn loop: the cube is left in the world and
# each run the camera finds it wherever the PREVIOUS run dropped it, picks it, and
# drops it at a NEW safe spot. So the destination can no longer be one fixed point
# ('table' at (0.35,0.2) would just place the cube back on itself next run). These
# are a few pre-vetted drop slots, all:
#   - inside the arm's demonstrated-reachable zone (x in [0.35,0.55],
#     y in [-0.20,0.20]); the overhead camera at (0.4,0,1.2), 60 deg FOV, easily
#     covers this, so reach (not view) is the binding constraint;
#   - on the ground at the cube's resting-center z (0.015), so object_detector's
#     center-z plausibility gate (0.015 +/- 0.04) still accepts the placed cube and
#     the NEXT run can perceive it (a cube on ANY face still centers at 0.015);
#   - mutually well separated (~0.18 m), so every move is genuine and the next
#     detection is unambiguous.
# TUNABLE: ground-truth verification confirms each slot stays in reach.
PLACE_SLOTS = [
    (0.45,  0.18, 0.015),
    (0.50,  0.00, 0.015),
    (0.45, -0.18, 0.015),
    (0.37,  0.00, 0.015),
]


def choose_place_target(current_xyz, logger=None):
    """Pick the next drop slot for a cube currently at `current_xyz`.

    Stateless ring-tour: find the slot NEAREST the cube now, return the NEXT slot
    in PLACE_SLOTS order. Across independent `pick_and_place` processes this walks
    the cube around the ring (slot i -> i+1 -> ...), so every run moves it to a
    fresh, known-safe, in-view, in-reach spot WITHOUT any persisted state and
    WITHOUT ever placing it back where it already is. If the cube's position is
    unknown (e.g. pick could not report it), start the tour at slot 0.
    """
    if current_xyz is None:
        idx = 0
    else:
        cx, cy = current_xyz[0], current_xyz[1]
        nearest = min(range(len(PLACE_SLOTS)),
                      key=lambda i: (cx - PLACE_SLOTS[i][0]) ** 2
                      + (cy - PLACE_SLOTS[i][1]) ** 2)
        idx = (nearest + 1) % len(PLACE_SLOTS)
    target = PLACE_SLOTS[idx]
    if logger is not None:
        cur = tuple(round(v, 3) for v in current_xyz) if current_xyz else None
        logger.info(f"choose_place_target: cube at {cur} -> drop slot {idx} @ "
                    f"({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})")
    return target


class _ObjectPoseCache:
    """Owns a subscription to /detected_objects/box and holds the latest pose.

    S20 FIX: the callback only fires when the twin node is SPUN. Nothing spins
    twin between setup() and the first resolve_object call, so a passive cache
    stayed empty -> pick failed with "no fresh pose" even though the detector was
    publishing at 10 Hz. So get_fresh() now actively pumps the twin executor
    (exactly like _read_finger_width) until a pose newer than its request arrives
    or a short deadline passes. Fresh pose -> xyz; nothing within the wait ->
    None, still honest (no made-up coordinate).
    """
    def __init__(self, node, logger):
        self._node = node
        self._logger = logger
        self._latest = None          # (stamp_sec, (x, y, z))
        node.create_subscription(PoseStamped, _DETECT_TOPIC, self._on_pose, 10)
        logger.info(f"resolve_object: subscribed to live perception on {_DETECT_TOPIC}")

    def _on_pose(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._latest = (t, (msg.pose.position.x,
                            msg.pose.position.y,
                            msg.pose.position.z))

    def get_fresh(self, now_sec=None, wait=2.5):
        """Spin twin up to `wait` seconds to catch a NEW pose.

        Freshness is guaranteed by construction, not by comparing timestamps:
        we clear _latest first, then wait for the next publish. Whatever arrives
        was published DURING this call, so it is fresh regardless of whose clock
        stamped it. (An earlier version compared msg.header.stamp against the
        caller's clock, but the detector stamps wall-time while move_to reads
        sim-time -- the mismatch made a valid pose look 1.7 billion seconds old.
        Clock-independent freshness sidesteps that entirely.) `now_sec` is now
        unused; kept in the signature so callers don't change.
        """
        exec_ = SingleThreadedExecutor()
        exec_.add_node(self._node)
        try:
            self._latest = None       # force a genuinely new sample
            deadline = time.time() + wait
            while time.time() < deadline:
                exec_.spin_once(timeout_sec=0.1)
                if self._latest is not None:
                    break
            if self._latest is None:
                self._logger.warn(
                    f"resolve_object: no pose on {_DETECT_TOPIC} within {wait}s "
                    f"of spinning; detector not publishing or cube out of view")
                return None
            _, xyz = self._latest      # arrived during the wait -> fresh
            return xyz
        finally:
            exec_.remove_node(self._node)


# Created in setup() once the twin node exists.
_POSE_CACHE = None


def resolve_object(object_id, logger, twin=None):
    """object_id -> (x, y, z) resting center in world/panda_link0 frame, or None.

    S20: 'box' comes from LIVE perception (the overhead camera via _POSE_CACHE).
    Place targets ('table') stay fixed. Same (x, y, z) | None contract as the old
    hardcoded version, so pick()/place() are untouched apart from passing `twin`.
    """
    # Place targets: fixed coordinates, no perception involved.
    if object_id in _KNOWN_TARGETS:
        return _KNOWN_TARGETS[object_id]

    # Perceived objects: live pose.
    if object_id == "box":
        if _POSE_CACHE is None:
            logger.error("resolve_object: perception not initialised "
                         "(setup() did not create the pose cache)")
            return None
        xyz = _POSE_CACHE.get_fresh()   # clock-independent: waits for a new pose
        if xyz is None:
            logger.error(
                "resolve_object: no fresh camera pose for 'box'. Is the cube in "
                "view and object_detector running? (Nothing detected -> not "
                "driving to a guessed point.)")
            return None
        logger.info(f"resolve_object: live 'box' @ "
                    f"({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})")
        return xyz

    logger.error(f"resolve_object: unknown object_id '{object_id}' "
                 f"(perceived: ['box'], targets: {sorted(_KNOWN_TARGETS)})")
    return None


@dataclass
class PickResult:
    """Composite result for pick(). Reports which stage ended it and why, so a
    caller (self-correction, task sequencing, the LLM verb) can react. `held`
    mirrors the grasp predicate: True only if the gripper actually stalled on the
    object. `object_xyz` echoes what pick drove to (diagnostic)."""
    ok: bool
    stage: str          # "picked" | "no_object_resolved" | "ready_pose_failed"
                        # | "approach_failed" | "descend_failed"
                        # | "no_object_grasped" | "lift_failed" | "dropped_in_lift"
                        # | "grip_error" | "grasp_width_implausible"
    held: bool = False
    object_xyz: Optional[Tuple[float, float, float]] = None
    error: Optional[str] = None

    def __str__(self):
        if self.ok:
            x, y, z = self.object_xyz if self.object_xyz else (0, 0, 0)
            return f"ok [picked] object@({x:.3f},{y:.3f},{z:.3f}) HELD"
        return f"FAILED [{self.stage}]: {self.error}"


@dataclass
class PlaceResult:
    """Composite result for place(). Mirrors PickResult architecture. Reports
    which stage ended it and why, so a caller (task sequencing, LLM verb) can
    react. `released` is True only if the gripper successfully opened and the
    object was released (fingers returned to ~GRIPPER_MAX). `target_xyz` echoes
    the place target (diagnostic)."""
    ok: bool
    stage: str          # "placed" | "no_target_resolved" | "not_holding"
                        # | "ready_pose_failed" | "approach_failed"
                        # | "descend_failed" | "open_failed" | "not_released"
                        # | "retract_failed"
    released: bool = False
    target_xyz: Optional[Tuple[float, float, float]] = None
    error: Optional[str] = None

    def __str__(self):
        if self.ok:
            x, y, z = self.target_xyz if self.target_xyz else (0, 0, 0)
            return f"ok [placed] target@({x:.3f},{y:.3f},{z:.3f}) RELEASED"
        return f"FAILED [{self.stage}]: {self.error}"


def pick(robot, arm, logger, params, twin, object_id,
         expect_half_width=None, width_tol=None, home_first=True,
         blend=False) -> PickResult:
    """Composite: resolve object -> approach (top-down) -> open -> descend ->
    close -> confirm STALL -> lift. First real consumer of BOTH MoveResult and
    the grasp signal. Every sub-step returns a rich result and pick reacts to it:
    a failed plan aborts honestly; a grip that merely `reached` (no stall) means
    nothing was grasped -> abort as 'no_object_grasped' rather than lifting air.

    Preconditions baked in from S16 §3:
      - object xyz is in panda_link0 frame == world frame (identity).
      - the top-down TOPDOWN_GRASP_QUAT cancels the hand's -pi/4 yaw, so the cube
        spawns axis-aligned (no -Y) and meets the fingers on flat faces.
      - approach/lift heights stay inside the §3b reachable box.
    """
    logger.info(f"=== pick('{object_id}') ===")

    q = TOPDOWN_GRASP_QUAT

    # 0. Home to the 'ready' named pose FIRST, every pick. This is the fix for
    # S17 run 2: approach IK was solved from wherever the arm happened to be, and
    # the planner picked an ugly branch that pinned panda_joint2 at its limit
    # (1.7628) -- so the lift then failed CheckStartStateBounds ("start state out
    # of bounds"). Starting every pick from the same well-conditioned, elbow-up
    # 'ready' configuration gives the planner a consistent basin, so it finds
    # solutions with joint headroom and the descend doesn't strand a joint at its
    # limit. This is the "universal pre-pick posture" -- standard industrial
    # practice: return to a repeatable home before each cycle so it's deterministic.
    # Phase 2: callers that guarantee the arm is already at a well-conditioned
    # pose (e.g. parcel_task homes to 'ready' before each pick) pass
    # home_first=False to skip this redundant re-home. Default True preserves the
    # original behaviour for every other caller.
    if home_first:
        logger.info("pick: home to 'ready' pose")
        r = move_to_named(robot, arm, logger, params, "ready")
        if not r.ok:
            return PickResult(ok=False, stage="ready_pose_failed", object_xyz=None,
                              error=f"could not reach 'ready' pose: {r}")

    # S21: resolve AFTER homing. The arm used to be left parked over the cube by
    # the previous cycle, and resolving first read the camera while the hand
    # occluded it (-> a pose taken from arm pixels, z=0.43/0.70). From 'ready' the
    # arm is out of the overhead camera's view of the pick area.
    # A4: object_id may be an explicit (x, y, z). The sort task must pick a SPECIFIC
    # detected object, not "whatever the legacy single-object topic reports as largest".
    # Symmetric with place(), which already accepts a tuple.
    if isinstance(object_id, (tuple, list)) and len(object_id) == 3:
        xyz = (float(object_id[0]), float(object_id[1]), float(object_id[2]))
    else:
        xyz = resolve_object(object_id, logger, twin)   # S20: twin -> pose freshness
        if xyz is None:
            return PickResult(ok=False, stage="no_object_resolved",
                              error=f"could not resolve object '{object_id}'")
    ox, oy, oz = xyz
    z_grasp = oz + FINGER_REACH_Z                 # link8 height to put fingers around it
    z_approach = z_grasp + APPROACH_CLEARANCE     # pre-grasp, clear above
    z_lift = z_grasp + LIFT_HEIGHT                # raise with object in hand

    # 1-3. Approach above the object, open the gripper, and descend to grasp.
    # Phase 3 (blend=True): open the gripper FIRST -- harmless to do early, and it
    # lets the approach and descend run as ONE continuous re-timed motion that only
    # stops at the grasp (the "move to object, then grab" the user wants). If the
    # blended chain can't be planned/executed it returns falsy and we fall through
    # to the original discrete approach -> descend, so behaviour is never worse.
    blended = False
    if blend:
        g = move_gripper(robot, arm, logger, params, twin, GRIPPER_MAX)
        if not g.ok:
            return PickResult(ok=False, stage="grip_error", object_xyz=xyz,
                              error=f"pre-grasp open failed: {g}")
        a = _grasp_goal_state(robot, ox, oy, z_approach, q, logger)
        d = _grasp_goal_state(robot, ox, oy, z_grasp, q, logger)
        if a is not None and d is not None:
            logger.info(f"pick: blended approach+descend to grasp z={z_grasp:.3f}")
            res = _plan_chain_execute(robot, arm, logger, params, [a, d])
            blended = res is not None and res.ok

    if not blended:
        # 1. Approach: move above the object, hand already in grasp orientation.
        logger.info(f"pick: approach ({ox:.3f},{oy:.3f},{z_approach:.3f}) top-down")
        r = move_to_grasp_pose(robot, arm, logger, params, ox, oy, z_approach, q)
        if not r.ok and not home_first:
            # Direct approach from the current pose failed; recover the old
            # reliability by homing to the 'ready' basin and retrying once.
            logger.warn("pick: direct approach failed; falling back to home-then-approach")
            if move_to_named(robot, arm, logger, params, "ready").ok:
                r = move_to_grasp_pose(robot, arm, logger, params, ox, oy, z_approach, q)
        if not r.ok:
            return PickResult(ok=False, stage="approach_failed", object_xyz=xyz,
                              error=f"approach plan failed: {r}")

        # 2. Open the gripper before descending (skipped if blend already opened it).
        if not blend:
            g = move_gripper(robot, arm, logger, params, twin, GRIPPER_MAX)
            if not g.ok:
                return PickResult(ok=False, stage="grip_error", object_xyz=xyz,
                                  error=f"pre-grasp open failed: {g}")

        # 3. Descend straight down to grasp height (same orientation). A vertical
        # descent doesn't sweep through the object's xy footprint, so it does not
        # nudge the object.
        logger.info(f"pick: descend to grasp z={z_grasp:.3f}")
        r = move_to_grasp_pose(robot, arm, logger, params, ox, oy, z_grasp, q)
        if not r.ok:
            return PickResult(ok=False, stage="descend_failed", object_xyz=xyz,
                              error=f"descend plan failed: {r}")

    # 4. Close on the object. The grasp predicate is GripResult.stalled:
    #    stalled==True -> held; stalled==False (merely 'reached') -> the fingers
    #    closed all the way with nothing between them = nothing grasped.
    g = move_gripper(robot, arm, logger, params, twin, GRIPPER_MIN)
    if not g.ok:
        return PickResult(ok=False, stage="grip_error", object_xyz=xyz,
                          error=f"grasp close failed: {g}")
    if not g.stalled:
        # 'reached' means fingers met with nothing between them. The width tells
        # you WHY at a glance: ~0.0 = empty (fingers above/beside the object, or
        # object gone); a moderate stall-less close near the cube size would be
        # odd and worth a look. S17 run 1: width 0.0098 = closed on air.
        logger.warn(f"pick: gripper {g.stage} at width={g.final_width:.4f} "
                    f"(no stall) -> nothing grasped")
        return PickResult(ok=False, stage="no_object_grasped", object_xyz=xyz,
                          error=f"gripper reached width={g.final_width:.4f} "
                                f"without stalling; no object between the fingers")

    # S21: SIZE PLAUSIBILITY. "Stopped short of the commanded close" was treated as
    # proof of an object, but anything that stops the fingers looks the same: with
    # the fingertips dragging on the ground the hand closed only to a mean width of
    # 0.0365 (the 3 cm cube seats at 0.0151), the hold stroke "compressed" against
    # whatever was jamming it, and pick() lifted an EMPTY hand and later reported
    # place=True over a cube that never moved. The target is a known 0.03 m cube, so
    # a seated width far from CUBE_HALF_WIDTH is not this cube. Fail honestly.
    # The gate expectation defaults to the ground cube (backward compatible); a caller
    # with a differently sized object (e.g. the 40 mm sorting-line box seats at ~0.020)
    # passes expect_half_width/width_tol. The gate itself is NOT relaxed -- it is retargeted
    # to the known object; it still refuses a jam/empty grasp (S21).
    ehw = CUBE_HALF_WIDTH if expect_half_width is None else expect_half_width
    wtol = GRASP_WIDTH_TOL if width_tol is None else width_tol
    if g.final_width is None or abs(g.final_width - ehw) > wtol:
        logger.warn(f"pick: 'stalled' at width={g.final_width} but a {2*ehw:.3f} m "
                    f"object seats at {ehw:.4f} +/- {wtol}; this is not the "
                    f"object (fingers jammed?) -> NOT lifting")
        return PickResult(ok=False, stage="grasp_width_implausible", object_xyz=xyz,
                          error=f"stalled at width={g.final_width}, expected "
                                f"{ehw}+/-{wtol}")

    logger.info(f"pick: grasp confirmed (stalled) width={g.final_width}")
    grasp_width = g.final_width  # remember where the object seated the fingers

    # 5. Lift the object.
    logger.info(f"pick: lift to z={z_lift:.3f}")
    r = move_to_grasp_pose(robot, arm, logger, params, ox, oy, z_lift, q)
    if not r.ok:
        # Grasped but couldn't lift -- report honestly; object is still held.
        return PickResult(ok=False, stage="lift_failed", held=True,
                          object_xyz=xyz, error=f"lift plan failed: {r}")

    # 6. RE-VERIFY the grasp after lifting. The stall at step 4 only proves we
    # grasped BEFORE lifting; the object can still slip out mid-lift (weak grip,
    # jolt). Re-read the finger width: if the fingers have now closed to ~empty,
    # the object was dropped and 'picked' would be a lie. Only report success if
    # the fingers are STILL pinching near where the object seated them.
    live = _read_finger_width(twin, logger)
    if live is not None and grasp_width is not None:
        # Fingers closing well past the grasp point (toward empty) => dropped.
        if live < grasp_width - GRASP_DROP_MARGIN:
            logger.warn(f"pick: post-lift width={live:.4f} << grasp {grasp_width:.4f} "
                        f"-> object slipped out during lift")
            # S20: the drop is detected HERE, not in move_gripper, so this is the
            # one loss-of-grasp the record's normal writer never sees. Without this
            # the record would still say HOLDING and a following place() would run
            # a full cycle on an empty hand.
            _clear_grasp(logger, "object dropped during lift")
            return PickResult(ok=False, stage="dropped_in_lift", held=False,
                              object_xyz=xyz,
                              error=f"grasp lost during lift (fingers now "
                                    f"{live:.4f}, grasped at {grasp_width:.4f})")

    res = PickResult(ok=True, stage="picked", held=True, object_xyz=xyz)
    logger.info(f"pick('{object_id}') -> {res}")
    return res


def place(robot, arm, logger, params, twin, target_id, clearance=0.0,
          home_first=True, blend=False, before_release=None,
          release_seconds=None, retract=True) -> PlaceResult:
    """Composite: resolve target -> approach (top-down) -> descend -> open ->
    confirm RELEASE -> retract. Mirror of pick() with opposite gripper action.

    Preconditions:
      - The arm is currently holding an object (caller's responsibility; we
        pre-check that the gripper width is NOT empty).
      - target_id resolves via resolve_object (same perception seam as pick).
      - Approach/descend/retract use the same top-down TOPDOWN_GRASP_QUAT as pick,
        so the target location is treated with the same alignment (axis-aligned).
    """
    # S22: target_id may be an explicit (x, y, z) drop point (from
    # choose_place_target) OR a named target id resolved via the perception seam.
    if isinstance(target_id, (tuple, list)) and len(target_id) == 3:
        txyz = (float(target_id[0]), float(target_id[1]), float(target_id[2]))
        logger.info(f"=== place(xyz=({txyz[0]:.3f},{txyz[1]:.3f},{txyz[2]:.3f})) ===")
    else:
        logger.info(f"=== place('{target_id}') ===")
        # Resolve the target location.
        txyz = resolve_object(target_id, logger, twin)   # S20: twin -> clock for pose freshness
        if txyz is None:
            # S19 FIX: was mislabelled "not_holding", which sent you looking at the
            # gripper for what is actually a bad target id.
            return PlaceResult(ok=False, stage="no_target_resolved",
                               error=f"could not resolve target '{target_id}' "
                                     f"(known targets: {sorted(_KNOWN_TARGETS)})")
    tx, ty, tz = txyz

    # PRE-CHECK: are we holding something? Read the current gripper width.
    # If it's close to GRIPPER_MAX (~0.04), the gripper is open -> not holding.
    # S20: judged against the grasp record, using GRASP_RELEASE_MARGIN -- the SAME
    # constant step 4 uses to declare a release. So "released" now implies "not
    # holding" by construction; the two can no longer contradict each other.
    current_width = _read_finger_width(twin, logger)
    recorded = _GRASP_STATE["width"] if _GRASP_STATE["known"] else None
    empty = None
    if _GRASP_STATE["known"] and recorded is None:
        empty = "the last gripper move ended holding nothing"
    elif recorded is not None and current_width is not None:
        if current_width - recorded >= GRASP_RELEASE_MARGIN:
            empty = (f"fingers are {current_width - recorded:.4f} wider than the "
                     f"recorded grasp ({recorded:.4f}) -- already released")
    elif current_width is not None:
        # No grasp seen in this process (e.g. `move_to place table` run on its own
        # after a separate pick). Absolute test is all we have -- flag it as weak
        # rather than letting an unverified pass look like a verified one.
        logger.warn("place: no grasp recorded in this process; falling back to the "
                    "absolute width test (UNVERIFIED -- cannot distinguish a hand "
                    "holding an object from one merely left part-closed)")
        if current_width > GRIPPER_MAX - GRIPPER_EMPTY_MARGIN:
            empty = f"width {current_width:.4f} is commanded-open"

    if empty is not None:
        logger.warn(f"place: not holding anything -- {empty}; nothing to place")
        return PlaceResult(ok=False, stage="not_holding",
                           error=f"not holding an object ({empty})")

    # Place geometry (mirrors pick's).
    # A4: `clearance` raises the release height. Placing INTO a walled bin by reaching
    # to the bin floor fails: once the fingers open to 0.08 m span (~0.10 padded) inside
    # a 0.132 m interior, MoveIt finds panda_leftfinger/rightfinger in contact with the
    # bin wall, so the retract path is invalid at its first states and place() returns
    # retract_failed -- even though the object was released correctly. Releasing just
    # ABOVE the rim (fingers clear of the 0.03 m wall band) removes the conflict
    # entirely, and is how bin-placing is done in practice. 0 for open surfaces.
    z_place = tz + FINGER_REACH_Z + clearance   # link8 height to release fingers
    z_approach = z_place + APPROACH_CLEARANCE   # pre-place, clear above
    z_retract = z_place + LIFT_HEIGHT           # post-release, retract height
    q = TOPDOWN_GRASP_QUAT

    # 0. Optionally home to 'ready' for a well-conditioned planning basin. Phase 2:
    # a caller that just lifted an object from a pick (parcel_task) passes
    # home_first=False so the arm routes DIRECTLY object->tray instead of detouring
    # all the way home and back down. Default True preserves the original path for
    # every other caller.
    if home_first:
        logger.info("place: home to 'ready' pose")
        r = move_to_named(robot, arm, logger, params, "ready")
        if not r.ok:
            return PlaceResult(ok=False, stage="ready_pose_failed", target_xyz=txyz,
                               error=f"could not reach 'ready' pose: {r}")

    # 1-2. Approach above the target and descend to place height. Phase 3
    # (blend=True): run approach+descend as ONE continuous re-timed motion that
    # only stops at the release point. Falls through to discrete moves on failure.
    blended = False
    if blend:
        a = _grasp_goal_state(robot, tx, ty, z_approach, q, logger)
        d = _grasp_goal_state(robot, tx, ty, z_place, q, logger)
        if a is not None and d is not None:
            logger.info(f"place: blended approach+descend to place z={z_place:.3f}")
            res = _plan_chain_execute(robot, arm, logger, params, [a, d])
            blended = res is not None and res.ok

    if not blended:
        # 1. Approach: move above the target, hand in same top-down orientation.
        logger.info(f"place: approach ({tx:.3f},{ty:.3f},{z_approach:.3f}) top-down")
        r = move_to_grasp_pose(robot, arm, logger, params, tx, ty, z_approach, q)
        if not r.ok and not home_first:
            # Direct object->tray approach failed; fall back to home-then-approach
            # so the old reliability is preserved (object is still held throughout).
            logger.warn("place: direct approach failed; falling back to home-then-approach")
            if move_to_named(robot, arm, logger, params, "ready").ok:
                r = move_to_grasp_pose(robot, arm, logger, params, tx, ty, z_approach, q)
        if not r.ok:
            return PlaceResult(ok=False, stage="approach_failed", target_xyz=txyz,
                               error=f"approach plan failed: {r}")

        # 2. Descend to place height (same orientation).
        logger.info(f"place: descend to place z={z_place:.3f}")
        r = move_to_grasp_pose(robot, arm, logger, params, tx, ty, z_place, q)
        if not r.ok:
            return PlaceResult(ok=False, stage="descend_failed", target_xyz=txyz,
                               error=f"descend plan failed: {r}")

    # Remember how tight the hand was BEFORE opening, so release can be judged
    # relative to the grasp (fingers moved clearly open) rather than against a
    # hard 0.04 the loaded open stroke may not perfectly reach.
    grasp_width_before_open = _read_finger_width(twin, logger)

    # 3. Open the gripper to release the object. This is the inverse of pick's
    #    close: we move to GRIPPER_MAX (full open).
    if before_release is not None:
        before_release()
    g = move_gripper(robot, arm, logger, params, twin, GRIPPER_MAX, seconds=release_seconds)
    if not g.ok:
        return PlaceResult(ok=False, stage="open_failed", target_xyz=txyz,
                           error=f"gripper open failed: {g}")

    # 4. RELEASE CONFIRMATION.
    #    S19b: judge release RELATIVELY. A released cube is one the fingers no
    #    longer touch -- i.e. they opened CLEARLY WIDER than the grasp width. We
    #    don't require hitting exactly 0.04: a position-mode JTC reversing a
    #    settled hold may finish at 0.038, or still be creeping when SETTLE_MAX
    #    caps it, and both are real releases as long as the hand let go of a
    #    ~15mm cube. RELEASE_MARGIN is the "clearly wider" threshold. If the
    #    fingers barely moved from the grasp width, the object is still pinched.
    if g.final_width is None or grasp_width_before_open is None:
        logger.warn("place: could not read finger width around open; "
                    "assuming release (unverified)")
    elif g.final_width - grasp_width_before_open < GRASP_RELEASE_MARGIN:
        logger.warn(f"place: fingers moved only "
                    f"{g.final_width - grasp_width_before_open:.4f} open "
                    f"(from {grasp_width_before_open:.4f} to {g.final_width:.4f}); "
                    f"< {GRASP_RELEASE_MARGIN} -> object may not have released")
        return PlaceResult(ok=False, stage="not_released", target_xyz=txyz,
                           error=f"fingers opened only "
                                 f"{g.final_width - grasp_width_before_open:.4f} "
                                 f"past the grasp; object may still be in the hand")
    else:
        logger.info(f"place: release confirmed -- opened from "
                    f"{grasp_width_before_open:.4f} to {g.final_width:.4f}")

    # 5. Retract the arm to clear the placed object. A caller that immediately
    # plans its own departure from the release pose (retract=False) skips this
    # stop: the release point already sits `clearance` above the tote rim.
    if not retract:
        res = PlaceResult(ok=True, stage="placed", released=True, target_xyz=txyz)
        logger.info(f"place('{target_id}') -> {res} (no separate retract)")
        return res
    logger.info(f"place: retract to z={z_retract:.3f}")
    r = move_to_grasp_pose(robot, arm, logger, params, tx, ty, z_retract, q)
    if not r.ok:
        # Object was placed but we couldn't retract cleanly -- report honestly.
        return PlaceResult(ok=False, stage="retract_failed", released=True,
                           target_xyz=txyz, error=f"retract plan failed: {r}")

    res = PlaceResult(ok=True, stage="placed", released=True, target_xyz=txyz)
    logger.info(f"place('{target_id}') -> {res}")
    return res


def pick_and_place(robot, arm, logger, params, twin, pick_id, place_id=None):
    """Composite: pick() -> place() at an AUTO-CHOSEN safe slot -> park at 'ready'.

    S22: the drop location is chosen each run by choose_place_target() from where
    the cube was actually picked, so repeated cycles need NO respawn -- the camera
    finds the cube wherever the last run left it, and it is moved to a fresh safe
    slot (touring the ring across runs). `place_id` is accepted for CLI/back-compat
    but ignored UNLESS it names a fixed _KNOWN_TARGETS entry other than the default
    'table' (pass e.g. a future named bin to force a specific drop point).

    Returns (pick_result, place_result); either may be not ok. The arm is homed to
    'ready' at the end REGARDLESS of outcome (Feature C), which also clears it out
    of the overhead camera's view so the next cycle's detection sees the cube.
    """
    logger.info(f"=== pick_and_place('{pick_id}' -> auto) ===")

    place_result = None
    try:
        # Phase 1: pick
        pick_result = pick(robot, arm, logger, params, twin, pick_id)
        logger.info(f"pick phase: {pick_result}")
        if not pick_result.ok:
            logger.error(f"pick failed at stage {pick_result.stage}; "
                         f"aborting place phase")
            return (pick_result, None)

        # Phase 2: choose a safe drop slot from where the cube actually was, place there.
        if place_id not in (None, "table") and place_id in _KNOWN_TARGETS:
            target = place_id                                  # explicit fixed target
        else:
            target = choose_place_target(pick_result.object_xyz, logger)
        place_result = place(robot, arm, logger, params, twin, target)
        logger.info(f"place phase: {place_result}")
        logger.info(f"pick_and_place cycle: {'SUCCESS' if place_result.ok else 'FAILED'}")
        return (pick_result, place_result)
    finally:
        # Phase 3 (Feature C): return to a safe configuration every cycle, success
        # or fail. Non-fatal if it hiccups -- a homing error must not mask a place
        # result that already happened.
        try:
            logger.info("pick_and_place: returning to safe 'ready' configuration")
            move_to_named(robot, arm, logger, params, "ready")
        except Exception as e:
            logger.warn(f"pick_and_place: return-to-ready raised: {e}")


def main():
    argv = [a for a in sys.argv[1:] if not a.startswith("--")]

    robot, arm, logger, params, twin = setup()  # SEAM 1

    cmd = argv[0] if argv else "demo"
    if cmd == "named":
        move_to_named(robot, arm, logger, params, argv[1])
    elif cmd == "pose":
        x, y, z = float(argv[1]), float(argv[2]), float(argv[3])
        move_to_pose(robot, arm, logger, params, x, y, z)
    elif cmd == "grip":
        # ros2 run twin_moveit_scripts move_to grip 0.0        # close
        # ros2 run twin_moveit_scripts move_to grip 0.04       # open
        w = float(argv[1])
        eff = float(argv[2]) if len(argv) > 2 else GRIPPER_DEFAULT_EFFORT
        move_gripper(robot, arm, logger, params, twin, w, max_effort=eff)
    elif cmd == "pick":
        # ros2 run twin_moveit_scripts move_to pick box
        obj = argv[1] if len(argv) > 1 else "box"
        logger.info(f"RESULT: {pick(robot, arm, logger, params, twin, obj)}")
    elif cmd == "place":
        # ros2 run twin_moveit_scripts move_to place table
        target = argv[1] if len(argv) > 1 else "table"
        logger.info(f"RESULT: {place(robot, arm, logger, params, twin, target)}")
    elif cmd == "pick_and_place":
        # ros2 run twin_moveit_scripts move_to pick_and_place box
        # S22: the drop location is auto-chosen each run (no respawn needed). A 2nd
        # arg naming a fixed _KNOWN_TARGETS entry (other than 'table') forces it.
        pick_id = argv[1] if len(argv) > 1 else "box"
        place_id = argv[2] if len(argv) > 2 else None
        pick_result, place_result = pick_and_place(robot, arm, logger, params,
                                                    twin, pick_id, place_id)
        logger.info(f"Cycle result: pick={pick_result.ok}, place={place_result.ok if place_result else 'N/A'}")
    else:
        for name in ("ready", "home"):
            move_to_named(robot, arm, logger, params, name)

    logger.info("Done.")
    shutdown(twin)  # SEAM 3


if __name__ == "__main__":
    main()

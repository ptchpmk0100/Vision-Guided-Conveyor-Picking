#!/usr/bin/env python3
"""Stage 0 sorting-line reach probe, with Gazebo ground truth as the verdict.

This is a free-space kinematic gate for the proposed raised sorting line.  It does
not require the final belt, table, trays, detector, or sorter.

Run in two clean, identically sourced shells.  Shell 1 brings up the existing
headless verification cell (world name ``empty``)::

  env -i HOME="$HOME" TERM=xterm \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/opt/ros/jazzy/bin \
    bash --norc --noprofile
  source /opt/ros/jazzy/setup.bash
  source /home/pmkhoi/ros_test_ws_mirror/install/setup.bash
  export GZ_VERSION=harmonic
  ros2 launch \
    /home/pmkhoi/ros_test_ws_mirror/tools/verify_industrial_headless.launch.py

Shell 2 runs the probe after the controllers are active::

  env -i HOME="$HOME" TERM=xterm \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/opt/ros/jazzy/bin \
    bash --norc --noprofile
  source /opt/ros/jazzy/setup.bash
  source /home/pmkhoi/ros_test_ws_mirror/install/setup.bash
  export GZ_VERSION=harmonic
  python3 /home/pmkhoi/ros_test_ws_mirror/tools/reach_probe_sorting_line.py \
    --passes 3

``--passes 2`` is the minimum accepted value.  Three is preferred.  The existing
verification launch may spawn its legacy ground cube at the ground-control point;
the raised candidates remain a free-space test and do not depend on that cube.

For each candidate this script:

* returns to ``ready`` and confirms the live gripper width is open;
* commands the same seeded-IK top-down posture used by the real pick primitive;
* exercises approach -> work height -> lift/retract; and
* reads both finger links and ``panda_link7`` from
  ``/world/empty/pose/info`` after every leg.

The midpoint of the two Gazebo finger poses is the independent grasp-point
measurement; ``panda_link8`` is a massless flange and is not published by Gazebo.
The first ground-control work leg calibrates the constant vector from commanded
link8 target to that midpoint; every other leg is compared through that vector.
Gazebo translation and hand-down geometry decide whether a posture was reached. A
MoveResult is retained only as an independent health signal: a planning,
execution, or arrival failure makes the overall result NO-GO even if Gazebo shows
that the link happened to finish inside tolerance.
"""

import argparse
from dataclasses import dataclass
import math
import re
import subprocess
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

from twin_moveit_scripts.move_to import (
    FINGER_REACH_Z as MOVE_TO_FINGER_REACH_Z,
    GRIPPER_MAX,
    GRIPPER_OPEN_TOL,
    TOPDOWN_GRASP_QUAT,
    move_gripper,
    move_to_grasp_pose,
    move_to_named,
    setup,
    shutdown,
)


# Proposed sorting-line geometry.  FINGER_REACH_Z is relative to the object
# centre; it must not have SURFACE_Z added to it a second time.
SURFACE_Z = 0.295
BOX_HALF_HEIGHT = 0.025
BOX_CENTER_Z = 0.320
FINGER_REACH_Z = 0.100
APPROACH = 0.100
LIFT = 0.150
PLACE_CLEARANCE = 0.035

PASSES = 2
TRANSLATION_TOL = 0.010
HAND_DOWN_XY_TOL = 0.010
HAND_DOWN_MIN_DROP = 0.150
OFFSET_Z_ABS_MIN = 0.030
OFFSET_Z_ABS_MAX = 0.090
OFFSET_XY_TOL = 0.010
GROUND_WORK_Z = 0.115
POSE_TOPIC = "/world/empty/pose/info"
LEFT_FINGER_NAME = "panda_leftfinger"
RIGHT_FINGER_NAME = "panda_rightfinger"
LINK7_NAME = "panda_link7"

Quaternion = Tuple[float, float, float, float]
XYZ = Tuple[float, float, float]


@dataclass(frozen=True)
class Pose3:
    xyz: XYZ
    quat: Quaternion


@dataclass(frozen=True)
class GroundTruthSample:
    finger_mid: XYZ
    link7: XYZ
    panda_frames: Tuple[str, ...]


@dataclass(frozen=True)
class Candidate:
    name: str
    kind: str
    x: float
    y: float
    stages: Tuple[Tuple[str, float], ...]


@dataclass
class LegRecord:
    candidate: str
    pass_no: int
    stage: str
    commanded_target: XYZ
    expected_mid: Optional[XYZ]
    motion_ok: bool
    motion_stage: str
    gt_sample: Optional[GroundTruthSample]
    translation_error: Optional[float]
    hand_down_xy_error: Optional[float]
    hand_vertical_drop: Optional[float]
    gt_ok: bool
    passed: bool
    calibration_pending: bool = False
    note: str = ""


@dataclass
class ReadyRecord:
    label: str
    motion_ok: bool
    motion_stage: str
    gt_sample: Optional[GroundTruthSample]
    translation_error: Optional[float]
    structure_error: Optional[float]
    gt_ok: bool
    passed: bool
    note: str = ""


def _pick_stages() -> Tuple[Tuple[str, float], ...]:
    work = BOX_CENTER_Z + FINGER_REACH_Z
    return (
        ("approach", work + APPROACH),
        ("work", work),
        ("lift", work + LIFT),
    )


def _tray_stages() -> Tuple[Tuple[str, float], ...]:
    work = BOX_CENTER_Z + FINGER_REACH_Z + PLACE_CLEARANCE
    return (
        ("approach", work + APPROACH),
        ("work", work),
        ("retract", work + LIFT),
    )


def _ground_stages() -> Tuple[Tuple[str, float], ...]:
    return (
        ("approach", GROUND_WORK_Z + APPROACH),
        ("work", GROUND_WORK_Z),
        ("lift", GROUND_WORK_Z + LIFT),
    )


CANDIDATES: Tuple[Candidate, ...] = (
    Candidate("BOX_red", "box", 0.560, +0.110, _pick_stages()),
    Candidate("BOX_green", "box", 0.560, 0.000, _pick_stages()),
    Candidate("BOX_yellow", "box", 0.560, -0.110, _pick_stages()),
    Candidate("TRAY_red", "tray", 0.450, +0.180, _tray_stages()),
    Candidate("TRAY_green", "tray", 0.450, -0.180, _tray_stages()),
    Candidate("TRAY_yellow", "tray", 0.370, 0.000, _tray_stages()),
    Candidate("CTRL_ground", "ground", 0.500, 0.000, _ground_stages()),
)


def _distance(a: XYZ, b: XYZ) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _vector(a: XYZ, b: XYZ) -> XYZ:
    return tuple(a[i] - b[i] for i in range(3))  # type: ignore[return-value]


def _parse_pose_info(text: str) -> Dict[str, Pose3]:
    """Parse Gazebo's protobuf-text Pose_V output.

    Gazebo omits scalar fields whose value is zero.  Missing position/quaternion
    components therefore deliberately default to 0.0 when a pose is flushed.
    """
    poses: Dict[str, Pose3] = {}
    current_name: Optional[str] = None
    position: Dict[str, float] = {}
    orientation: Dict[str, float] = {}
    saw_position = False
    saw_orientation = False
    section: Optional[str] = None

    def flush() -> None:
        nonlocal current_name, position, orientation
        nonlocal saw_position, saw_orientation, section
        if current_name is not None and saw_position and saw_orientation:
            poses[current_name] = Pose3(
                xyz=tuple(position.get(k, 0.0) for k in "xyz"),
                quat=tuple(orientation.get(k, 0.0) for k in "xyzw"),
            )
        current_name = None
        position = {}
        orientation = {}
        saw_position = False
        saw_orientation = False
        section = None

    value_re = re.compile(r"^(x|y|z|w):\s*(-?[0-9.eE+\-]+)$")
    name_re = re.compile(r'^name:\s*"([^"]+)"$')
    for raw in text.splitlines():
        line = raw.strip()
        match = name_re.match(line)
        if match:
            flush()
            current_name = match.group(1)
            continue
        if current_name is None:
            continue
        if line.startswith("position"):
            section = "position"
            saw_position = True
            continue
        if line.startswith("orientation"):
            section = "orientation"
            saw_orientation = True
            continue
        if line == "}":
            section = None
            continue
        match = value_re.match(line)
        if match and section == "position":
            position[match.group(1)] = float(match.group(2))
        elif match and section == "orientation":
            orientation[match.group(1)] = float(match.group(2))
    flush()
    return poses


def _bare_name(name: str) -> str:
    normalised = name.replace("::", "/")
    return normalised.split("/")[-1]


def _read_ground_truth(
        timeout: float, attempts: int = 2,
) -> Tuple[Optional[GroundTruthSample], str]:
    last_error = "no Gazebo sample attempted"
    for attempt in range(1, attempts + 1):
        try:
            proc = subprocess.run(
                ["gz", "topic", "-e", "-n", "1", "-t", POSE_TOPIC],
                capture_output=True,
                text=True,
                timeout=timeout,
                check=False,
            )
        except Exception as exc:
            last_error = f"gz topic raised {exc!r}"
        else:
            if proc.returncode != 0:
                detail = proc.stderr.strip() or "no stderr"
                last_error = f"gz topic rc={proc.returncode}: {detail}"
            elif not proc.stdout.strip():
                last_error = "gz topic returned no pose data"
            else:
                poses = _parse_pose_info(proc.stdout)
                panda_frames = tuple(sorted(
                    name for name in poses if _bare_name(name).startswith("panda_")
                ))
                by_bare: Dict[str, List[Tuple[str, Pose3]]] = {}
                for name, pose in poses.items():
                    by_bare.setdefault(_bare_name(name), []).append((name, pose))
                required = (LEFT_FINGER_NAME, RIGHT_FINGER_NAME, LINK7_NAME)
                missing = [name for name in required if name not in by_bare]
                ambiguous = {
                    name: [full for full, _pose in by_bare.get(name, [])]
                    for name in required if len(by_bare.get(name, [])) > 1
                }
                if missing:
                    # Fail loudly: never infer or substitute a belief-derived frame.
                    last_error = (
                        f"missing Gazebo frame(s) {missing}; "
                        f"panda frames actually seen={list(panda_frames)}"
                    )
                    return None, last_error
                elif ambiguous:
                    last_error = (
                        f"ambiguous Gazebo frame(s) {ambiguous}; "
                        f"panda frames actually seen={list(panda_frames)}"
                    )
                    return None, last_error
                else:
                    left = by_bare[LEFT_FINGER_NAME][0][1].xyz
                    right = by_bare[RIGHT_FINGER_NAME][0][1].xyz
                    link7 = by_bare[LINK7_NAME][0][1].xyz
                    finger_mid = tuple(
                        0.5 * (left[i] + right[i]) for i in range(3)
                    )
                    return GroundTruthSample(
                        finger_mid=finger_mid,  # type: ignore[arg-type]
                        link7=link7,
                        panda_frames=panda_frames,
                    ), "finger midpoint from Gazebo physics poses"
        if attempt < attempts:
            time.sleep(0.2)
    return None, last_error


def _motion_health(result) -> Tuple[bool, str, str]:
    if result is None:
        return False, "no_result", "motion helper returned None"
    ok = bool(getattr(result, "ok", False))
    stage = str(getattr(result, "stage", "unknown"))
    error = getattr(result, "error", None)
    return ok, stage, "" if error is None else str(error)


def _hand_geometry(sample: GroundTruthSample) -> Tuple[float, float, bool]:
    link7_to_mid = _vector(sample.finger_mid, sample.link7)
    hand_down_xy = math.hypot(link7_to_mid[0], link7_to_mid[1])
    vertical_drop = sample.link7[2] - sample.finger_mid[2]
    hand_down = (hand_down_xy <= HAND_DOWN_XY_TOL and
                 vertical_drop >= HAND_DOWN_MIN_DROP)
    return hand_down_xy, vertical_drop, hand_down


def _apply_calibration(record: LegRecord, offset: XYZ) -> None:
    """Score a captured Gazebo sample against commanded link8 + calibrated delta."""
    record.expected_mid = tuple(
        record.commanded_target[i] + offset[i] for i in range(3)
    )  # type: ignore[assignment]
    record.calibration_pending = False
    if record.gt_sample is None:
        record.gt_ok = False
        record.passed = False
        return
    record.translation_error = _distance(
        record.gt_sample.finger_mid, record.expected_mid
    )
    _xy, _drop, hand_down = _hand_geometry(record.gt_sample)
    record.gt_ok = record.translation_error <= TRANSLATION_TOL and hand_down
    record.passed = record.gt_ok and record.motion_ok
    if record.note == "awaiting ground-work offset calibration":
        record.note = ""


def _execute_leg(robot, arm, logger, params, candidate: Candidate,
                 pass_no: int, stage: str, z: float, settle: float,
                 gz_timeout: float,
                 calibration_offset: Optional[XYZ]) -> LegRecord:
    # This point is the virtual link8 target sent to MoveIt.  The independent
    # expected finger midpoint is this target plus the ground-calibrated offset.
    commanded_target = (candidate.x, candidate.y, z)
    result = move_to_grasp_pose(
        robot, arm, logger, params,
        candidate.x, candidate.y, z, TOPDOWN_GRASP_QUAT,
    )
    motion_ok, motion_stage, motion_note = _motion_health(result)
    time.sleep(settle)
    sample, gt_note = _read_ground_truth(gz_timeout)
    if sample is None:
        return LegRecord(
            candidate=candidate.name,
            pass_no=pass_no,
            stage=stage,
            commanded_target=commanded_target,
            expected_mid=None,
            motion_ok=motion_ok,
            motion_stage=motion_stage,
            gt_sample=None,
            translation_error=None,
            hand_down_xy_error=None,
            hand_vertical_drop=None,
            gt_ok=False,
            passed=False,
            calibration_pending=calibration_offset is None,
            note="; ".join(n for n in (motion_note, gt_note) if n),
        )
    hand_down_xy, vertical_drop, hand_down = _hand_geometry(sample)
    # Orientation proxy: the midpoint must be directly below published link7.
    # Finger quaternions are not compared because their frames have a fixed
    # rotation relative to the virtual grasp frame.
    record = LegRecord(
        candidate=candidate.name,
        pass_no=pass_no,
        stage=stage,
        commanded_target=commanded_target,
        expected_mid=None,
        motion_ok=motion_ok,
        motion_stage=motion_stage,
        gt_sample=sample,
        translation_error=None,
        hand_down_xy_error=hand_down_xy,
        hand_vertical_drop=vertical_drop,
        gt_ok=False,
        passed=False,
        calibration_pending=calibration_offset is None,
        note=(motion_note or "awaiting ground-work offset calibration"
              if calibration_offset is None else motion_note),
    )
    if calibration_offset is not None:
        _apply_calibration(record, calibration_offset)
    return record


def _skipped_leg(candidate: Candidate, pass_no: int, stage: str, z: float,
                 reason: str) -> LegRecord:
    return LegRecord(
        candidate=candidate.name,
        pass_no=pass_no,
        stage=stage,
        commanded_target=(candidate.x, candidate.y, z),
        expected_mid=None,
        motion_ok=False,
        motion_stage="skipped",
        gt_sample=None,
        translation_error=None,
        hand_down_xy_error=None,
        hand_vertical_drop=None,
        gt_ok=False,
        passed=False,
        calibration_pending=False,
        note=reason,
    )


def _check_ready(robot, arm, logger, params, label: str,
                 reference: Optional[GroundTruthSample], settle: float,
                 gz_timeout: float) -> ReadyRecord:
    result = move_to_named(robot, arm, logger, params, "ready")
    motion_ok, motion_stage, motion_note = _motion_health(result)
    time.sleep(settle)
    sample, gt_note = _read_ground_truth(gz_timeout)
    if sample is None:
        return ReadyRecord(
            label, motion_ok, motion_stage, None, None, None,
            False, False,
            "; ".join(n for n in (motion_note, gt_note) if n),
        )
    if reference is None:
        # The initial sample defines the Gazebo reference for named-pose
        # repeatability.  Motion health must still say the named command executed.
        return ReadyRecord(
            label, motion_ok, motion_stage, sample, 0.0, 0.0,
            True, motion_ok, motion_note or "Gazebo ready reference captured",
        )
    trans_error = _distance(sample.finger_mid, reference.finger_mid)
    structure_error = _distance(
        _vector(sample.finger_mid, sample.link7),
        _vector(reference.finger_mid, reference.link7),
    )
    gt_ok = (trans_error <= TRANSLATION_TOL and
             structure_error <= HAND_DOWN_XY_TOL)
    note = motion_note
    return ReadyRecord(
        label, motion_ok, motion_stage, sample, trans_error, structure_error,
        gt_ok, gt_ok and motion_ok, note,
    )


def _fmt_metric(value: Optional[float], scale: float, suffix: str) -> str:
    return "n/a" if value is None else f"{value * scale:.1f}{suffix}"


def _print_leg(record: LegRecord) -> None:
    gt = "PEND" if record.calibration_pending else (
        "PASS" if record.gt_ok else "FAIL"
    )
    motion = "OK" if record.motion_ok else "FAIL"
    overall = "PENDING" if record.calibration_pending else (
        "PASS" if record.passed else "FAIL"
    )
    trans = _fmt_metric(record.translation_error, 1000.0, "mm")
    down_xy = _fmt_metric(record.hand_down_xy_error, 1000.0, "mm")
    drop = _fmt_metric(record.hand_vertical_drop, 1000.0, "mm")
    print(
        f"PROBE pass={record.pass_no} pose={record.candidate:12s} "
        f"stage={record.stage:8s} GT={gt:4s} mid_err={trans:>8s} "
        f"down_xy={down_xy:>8s} drop={drop:>8s} "
        f"motion={motion:4s}({record.motion_stage}) -> {overall}"
    )
    if record.note:
        print(f"      note: {record.note}")
    sys.stdout.flush()


def _print_ready(record: ReadyRecord) -> None:
    gt = "PASS" if record.gt_ok else "FAIL"
    motion = "OK" if record.motion_ok else "FAIL"
    trans = _fmt_metric(record.translation_error, 1000.0, "mm")
    structure = _fmt_metric(record.structure_error, 1000.0, "mm")
    print(
        f"READY {record.label:20s} GT={gt:4s} "
        f"mid_delta={trans:>8s} structure_delta={structure:>8s} "
        f"motion={motion:4s}({record.motion_stage}) "
        f"-> {'PASS' if record.passed else 'FAIL'}"
    )
    if record.note:
        print(f"      note: {record.note}")
    sys.stdout.flush()


def _candidate_passes(candidate: Candidate, passes: int,
                      legs: Sequence[LegRecord],
                      preconditions: Dict[Tuple[str, int], bool]) -> bool:
    for pass_no in range(1, passes + 1):
        if not preconditions.get((candidate.name, pass_no), False):
            return False
        for stage, _z in candidate.stages:
            matching = [r for r in legs
                        if r.candidate == candidate.name
                        and r.pass_no == pass_no and r.stage == stage]
            if len(matching) != 1 or not matching[0].passed:
                return False
    return True


def _print_summary(passes: int, legs: Sequence[LegRecord],
                   ready_records: Sequence[ReadyRecord],
                   preconditions: Dict[Tuple[str, int], bool],
                   model_valid: bool) -> bool:
    print(f"\n===== SORTING-LINE RAISED REACH SUMMARY (passes={passes}) =====")
    print("pose          kind     approach        work             lift/retract     verdict")
    print("                         each cell: pass-count worst(mid/down-horizontal), mm")
    print("------------  -------  --------------  ---------------  ---------------  -------")
    candidate_results: Dict[str, bool] = {}
    for candidate in CANDIDATES:
        cells = []
        evaluated = candidate.kind == "ground" or model_valid
        for stage, _z in candidate.stages:
            rows = [r for r in legs
                    if r.candidate == candidate.name and r.stage == stage]
            if not evaluated:
                cells.append("NOT-EVALUATED")
                continue
            passed = sum(1 for r in rows if r.passed)
            worst_mm = max(
                (r.translation_error * 1000.0 for r in rows
                 if r.translation_error is not None),
                default=None,
            )
            worst_down_mm = max(
                (r.hand_down_xy_error * 1000.0 for r in rows
                 if r.hand_down_xy_error is not None),
                default=None,
            )
            if worst_mm is None or worst_down_mm is None:
                cells.append(f"{passed}/{passes} n/a")
            else:
                cells.append(
                    f"{passed}/{passes} {worst_mm:.1f}/{worst_down_mm:.1f}mm"
                )
        ok = (evaluated and
              _candidate_passes(candidate, passes, legs, preconditions))
        candidate_results[candidate.name] = ok
        verdict = "PASS" if ok else "FAIL" if evaluated else "NOT-EVAL"
        print(
            f"{candidate.name:12s}  {candidate.kind:7s}  "
            f"{cells[0]:14s}  {cells[1]:15s}  {cells[2]:15s}  "
            f"{verdict}"
        )

    ready_passed = sum(1 for r in ready_records if r.passed)
    print(
        f"CTRL_ready: {ready_passed}/{len(ready_records)} Gazebo-repeatable "
        f"and motion-healthy returns (initial sample is the reference)"
    )

    all_candidates_ok = model_valid and all(candidate_results.values())
    all_ready_ok = bool(ready_records) and ready_passed == len(ready_records)
    overall = all_candidates_ok and all_ready_ok

    failed = [c for c in CANDIDATES
              if (c.kind == "ground" or model_valid)
              and not candidate_results[c.name]]
    if not model_valid:
        print("\nFINGER-MID MODEL CHECK: FAIL")
        print(
            "- CTRL_ground could not produce a sane constant link8-target-to-"
            "finger-midpoint offset, or a repeated ground leg failed motion, "
            "hand-down, or 10 mm translation health. Raised poses were NOT "
            "EVALUATED."
        )
    elif failed:
        print("\nFallback indication (report only; no redesign performed):")
        if not candidate_results.get("CTRL_ground", False):
            print(
                "- Ground control failed: validate bringup, controllers, pose topic, "
                "and probe health before changing sorting-line geometry."
            )
        else:
            raised_failed = [c for c in failed if c.kind != "ground"]
            if raised_failed:
                print(
                    "- Raised candidates failed while the ground control passed: "
                    "probe the common surface at z=0.255, then z=0.215."
                )
            far_failed = any(c.x >= 0.550 for c in raised_failed)
            near_passed = any(
                c.x <= 0.450 and candidate_results[c.name]
                for c in CANDIDATES if c.kind != "ground"
            )
            if far_failed and near_passed:
                print(
                    "- Failures concentrate at large x: translate the whole layout "
                    "toward the base in 0.030 m steps and rerun every pose."
                )
            centre_box_ok = candidate_results.get("BOX_green", False)
            high_y_failed = any(
                abs(c.y) >= 0.100 and not candidate_results[c.name]
                for c in CANDIDATES if c.kind != "ground"
            )
            if high_y_failed and centre_box_ok:
                print(
                    "- High-|y| poses fail while the centre box passes: pull the "
                    "outer poses inward and rerun the complete layout."
                )
            if not (far_failed or high_y_failed):
                print(
                    "- Failures have no simple XY pattern: use the per-stage ground-"
                    "truth errors before choosing a geometric fallback."
                )

    print("===== FINAL: " + ("GO" if overall else "NO-GO") + " =====")
    if overall:
        print("All poses reached ground-truth tolerance with healthy motion results.")
    else:
        print("Do not build the final sorting-line scene from this probe result.")
    return overall


def _parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Ground-truth raised sorting-line reach probe"
    )
    parser.add_argument(
        "--passes", type=int, default=PASSES,
        help="passes per candidate (minimum 2; default: 2)",
    )
    parser.add_argument(
        "--settle", type=float, default=0.5,
        help="seconds to wait before each Gazebo pose sample (default: 0.5)",
    )
    parser.add_argument(
        "--gz-timeout", type=float, default=15.0,
        help="timeout per gz pose sample in seconds (default: 15)",
    )
    args = parser.parse_args(argv)
    if args.passes < 2:
        parser.error("--passes must be at least 2; one-shot probes lie")
    if args.settle < 0.0:
        parser.error("--settle must be non-negative")
    if args.gz_timeout <= 0.0:
        parser.error("--gz-timeout must be positive")
    return args


def run_probe(args: argparse.Namespace) -> bool:
    if not math.isclose(BOX_CENTER_Z, SURFACE_Z + BOX_HALF_HEIGHT,
                        abs_tol=1e-12):
        raise RuntimeError("BOX_CENTER_Z does not match surface + half-height")
    if not math.isclose(FINGER_REACH_Z, MOVE_TO_FINGER_REACH_Z,
                        abs_tol=1e-12):
        raise RuntimeError(
            "probe FINGER_REACH_Z differs from the real motion primitive: "
            f"{FINGER_REACH_Z} != {MOVE_TO_FINGER_REACH_Z}"
        )

    print("===== SORTING-LINE STAGE 0 RAISED REACH PROBE =====")
    print(
        f"surface={SURFACE_Z:.3f} box_center={BOX_CENTER_Z:.3f} "
        f"finger_reach={FINGER_REACH_Z:.3f} passes={args.passes}"
    )
    print(
        f"ground-truth midpoint tolerance: {TRANSLATION_TOL * 1000:.0f} mm; "
        f"hand-down: horizontal <= {HAND_DOWN_XY_TOL * 1000:.0f} mm and "
        f"vertical drop >= {HAND_DOWN_MIN_DROP * 1000:.0f} mm; "
        f"topic={POSE_TOPIC}"
    )
    print(
        "offset sanity: |dz|="
        f"{OFFSET_Z_ABS_MIN * 1000:.0f}..{OFFSET_Z_ABS_MAX * 1000:.0f} mm, "
        f"xy <= {OFFSET_XY_TOL * 1000:.0f} mm"
    )
    sys.stdout.flush()

    robot = arm = logger = params = twin = None
    legs: List[LegRecord] = []
    ready_records: List[ReadyRecord] = []
    preconditions: Dict[Tuple[str, int], bool] = {}
    try:
        robot, arm, logger, params, twin = setup(
            node_name="reach_probe_sorting_line"
        )

        initial_ready = _check_ready(
            robot, arm, logger, params, "initial-reference", None,
            args.settle, args.gz_timeout,
        )
        ready_records.append(initial_ready)
        _print_ready(initial_ready)
        ready_reference = initial_ready.gt_sample
        if ready_reference is None:
            print("Cannot establish Gazebo ready reference; aborting safely.")
            return _print_summary(
                args.passes, legs, ready_records, preconditions,
                model_valid=False,
            )

        def exercise(candidate: Candidate, pass_no: int,
                     calibration_offset: Optional[XYZ]) -> None:
            ready = _check_ready(
                robot, arm, logger, params,
                f"p{pass_no}:{candidate.name}", ready_reference,
                args.settle, args.gz_timeout,
            )
            ready_records.append(ready)
            _print_ready(ready)

            # Ground truth, not MoveResult.ok, decides whether it is safe to
            # continue from the requested ready configuration.
            physically_ready = ready.gt_ok and ready.gt_sample is not None
            grip = None
            open_ok = False
            open_note = ""
            if physically_ready:
                grip = move_gripper(
                    robot, arm, logger, params, twin, GRIPPER_MAX
                )
                width = getattr(grip, "final_width", None)
                open_ok = (
                    bool(getattr(grip, "ok", False))
                    and width is not None
                    and width >= GRIPPER_MAX - GRIPPER_OPEN_TOL
                )
                if width is None:
                    open_note = "live finger width unavailable"
                elif not open_ok:
                    open_note = (
                        f"gripper not confirmed open: width={width:.4f}, "
                        f"need >= {GRIPPER_MAX - GRIPPER_OPEN_TOL:.4f}, "
                        f"result={grip}"
                    )
            else:
                open_note = "ready pose not confirmed by Gazebo"

            pre_ok = ready.passed and open_ok
            preconditions[(candidate.name, pass_no)] = pre_ok
            if not open_ok:
                print(
                    f"OPEN pass={pass_no} pose={candidate.name:12s} "
                    f"-> FAIL ({open_note})"
                )
                for stage, z in candidate.stages:
                    record = _skipped_leg(
                        candidate, pass_no, stage, z,
                        f"precondition failed: {open_note}",
                    )
                    legs.append(record)
                    _print_leg(record)
                return
            print(
                f"OPEN pass={pass_no} pose={candidate.name:12s} "
                f"width={grip.final_width:.4f} -> PASS"
            )

            approach_safe = False
            for index, (stage, z) in enumerate(candidate.stages):
                if index > 0 and not approach_safe:
                    record = _skipped_leg(
                        candidate, pass_no, stage, z,
                        "approach was not reached in Gazebo; descend skipped "
                        "for safety",
                    )
                else:
                    record = _execute_leg(
                        robot, arm, logger, params, candidate, pass_no,
                        stage, z, args.settle, args.gz_timeout,
                        calibration_offset,
                    )
                legs.append(record)
                _print_leg(record)
                if index == 0:
                    # Before calibration, translation is intentionally pending;
                    # motion health plus a live sample and healthy hand-down
                    # geometry are sufficient to continue to ground calibration.
                    if calibration_offset is None:
                        approach_safe = (
                            record.motion_ok
                            and record.gt_sample is not None
                            and record.hand_down_xy_error is not None
                            and record.hand_down_xy_error <= HAND_DOWN_XY_TOL
                            and record.hand_vertical_drop is not None
                            and record.hand_vertical_drop >= HAND_DOWN_MIN_DROP
                        )
                    else:
                        approach_safe = record.passed

        # Calibrate the link8-to-finger-midpoint model at the verified ground
        # grasp before assigning any pass/fail verdict to raised poses.
        ground = next(c for c in CANDIDATES if c.kind == "ground")
        exercise(ground, 1, calibration_offset=None)
        calibration_work = [
            record for record in legs
            if record.candidate == ground.name
            and record.pass_no == 1 and record.stage == "work"
        ]
        calibration_offset: Optional[XYZ] = None
        calibration_sane = False
        if len(calibration_work) == 1:
            work = calibration_work[0]
            if (work.gt_sample is not None and work.motion_ok
                    and work.hand_down_xy_error is not None
                    and work.hand_down_xy_error <= HAND_DOWN_XY_TOL
                    and work.hand_vertical_drop is not None
                    and work.hand_vertical_drop >= HAND_DOWN_MIN_DROP):
                calibration_offset = _vector(
                    work.gt_sample.finger_mid, work.commanded_target
                )
                offset_xy = math.hypot(
                    calibration_offset[0], calibration_offset[1]
                )
                calibration_sane = (
                    offset_xy <= OFFSET_XY_TOL
                    and OFFSET_Z_ABS_MIN <= abs(calibration_offset[2])
                    <= OFFSET_Z_ABS_MAX
                )
                print(
                    "FINGER-MID CALIBRATION delta="
                    f"({calibration_offset[0] * 1000:+.1f}, "
                    f"{calibration_offset[1] * 1000:+.1f}, "
                    f"{calibration_offset[2] * 1000:+.1f}) mm "
                    f"xy={offset_xy * 1000:.1f} mm -> "
                    f"{'SANE' if calibration_sane else 'REJECTED'}"
                )
        if calibration_offset is None:
            print(
                "FINGER-MID CALIBRATION delta=unavailable: ground WORK needs "
                "motion=OK, a Gazebo sample, and healthy hand-down geometry."
            )

        if calibration_sane and calibration_offset is not None:
            # The first ground pass was captured before delta existed.  Score it
            # now; its work leg has zero translation error by construction.
            for record in legs:
                if record.candidate == ground.name and record.pass_no == 1:
                    _apply_calibration(record, calibration_offset)
                    print("CALIBRATED ", end="")
                    _print_leg(record)
            for pass_no in range(2, args.passes + 1):
                exercise(ground, pass_no, calibration_offset)

        model_valid = bool(
            calibration_sane
            and calibration_offset is not None
            and _candidate_passes(
                ground, args.passes, legs, preconditions
            )
        )
        if model_valid:
            print(
                "FINGER-MID MODEL CHECK: PASS -- ground WORK calibrated a sane "
                "constant offset and every ground leg passed; raised verdicts "
                "are enabled."
            )
            raised = [c for c in CANDIDATES if c.kind != "ground"]
            for pass_no in range(1, args.passes + 1):
                for candidate in raised:
                    exercise(candidate, pass_no, calibration_offset)
        else:
            print(
                "FINGER-MID MODEL CHECK: FAIL -- raised poses will be "
                "reported NOT-EVAL, not PASS/FAIL."
            )

        final_ready = _check_ready(
            robot, arm, logger, params, "final", ready_reference,
            args.settle, args.gz_timeout,
        )
        ready_records.append(final_ready)
        _print_ready(final_ready)
        return _print_summary(
            args.passes, legs, ready_records, preconditions, model_valid
        )
    finally:
        if twin is not None:
            shutdown(twin)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    try:
        return 0 if run_probe(args) else 1
    except KeyboardInterrupt:
        print("\n===== FINAL: NO-GO (interrupted) =====")
        return 130
    except Exception as exc:
        # A controlled NO-GO is more useful to the harness than an optimistic
        # partial result.  The manager's log-health scan will retain this detail.
        print(f"\nPROBE FATAL: {exc!r}")
        print("===== FINAL: NO-GO =====")
        return 1


if __name__ == "__main__":
    sys.exit(main())

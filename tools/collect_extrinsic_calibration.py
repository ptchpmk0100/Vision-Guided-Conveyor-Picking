#!/usr/bin/env python3
"""Collect B1 optical-point <-> Gazebo-world-top correspondences.

With the calibration launch already running headless, use a clean sourced shell:

  python3 /home/pmkhoi/ros_test_ws_mirror/tools/collect_extrinsic_calibration.py \
    --run-id b1_run_01 --target-half-height 0.002

Repeat from a fresh launch with ``--run-id b1_run_02``.  Datasets default to
``logs/v2/calibration/<run-id>.json`` under the mirror workspace.

This collector intentionally runs before any world ROI or expected-world-Z filter.  It
uses the production HSV/connected-component/median-depth measurement, deprojects with
CameraInfo.K, and uses the nominal extrinsic only for one-to-one identity association.
The nominal transform never enters the saved optical or truth correspondence values.
"""
import argparse
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


CAMERA_NS = "/overhead_camera"
POSE_TOPICS = (
    "/world/empty/dynamic_pose/info",
    "/world/empty/pose/info",
)
CAL_NAME = re.compile(r"^cal_(\d{2})_(red|green|yellow)_(box|tile)$")
EXPECTED_COUNT = 12
DEFAULT_TARGET_HALF_HEIGHT = 0.025

# Same segmentation gates as object_detector_sorting_line.py.  There is deliberately no
# world ROI or expected-Z gate here because the extrinsic is what B1 is solving.
SAT_MIN = 90
VAL_MIN = 50
MIN_AREA = 200
MAX_AREA = 1600
HUE_CLASSES = (
    ("red", lambda h: h <= 10 or h >= 170),
    ("yellow", lambda h: 20 <= h <= 35),
    ("green", lambda h: 40 <= h <= 85),
)

NOMINAL_R = np.array([
    [0.0, -1.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
], dtype=np.float64)
NOMINAL_T = np.array([0.4, 0.0, 1.2], dtype=np.float64)
ASSIGN_MAX_M = 0.030
UPRIGHT_MAX_DEG = 2.0
STABILITY_MAX_MPS = 0.001
CONDITIONING_MIN_RATIO = 0.01


class CollectionError(RuntimeError):
    pass


def _stamp_ns(msg: Image) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(
        msg.header.stamp.nanosec
    )


def _classify_colour(hue: float) -> Optional[str]:
    for name, test in HUE_CLASSES:
        if test(hue):
            return name
    return None


def _quat_matrix(q: Sequence[float]) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n <= 1e-12:
        raise CollectionError("Gazebo returned a zero quaternion")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def _top_world(pose: Dict[str, Sequence[float]],
               target_half_height: float) -> np.ndarray:
    position = np.asarray(pose["position"], dtype=np.float64)
    local_top = np.array([0.0, 0.0, target_half_height], dtype=np.float64)
    return position + _quat_matrix(pose["orientation"]) @ local_top


def _upright_deg(pose: Dict[str, Sequence[float]]) -> float:
    local_z_world = _quat_matrix(pose["orientation"])[:, 2]
    cosine = float(np.clip(local_z_world[2], -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def _parse_pose_text(text: str) -> Dict[str, Dict[str, List[float]]]:
    poses: Dict[str, Dict[str, List[float]]] = {}
    name: Optional[str] = None
    position: Dict[str, float] = {}
    orientation: Dict[str, float] = {}
    saw_position = False
    saw_orientation = False
    section: Optional[str] = None

    def flush() -> None:
        nonlocal name, position, orientation, saw_position, saw_orientation, section
        if name is not None and saw_position and saw_orientation:
            poses[name] = {
                "position": [position.get(k, 0.0) for k in "xyz"],
                "orientation": [orientation.get(k, 0.0) for k in "xyzw"],
            }
        name = None
        position = {}
        orientation = {}
        saw_position = False
        saw_orientation = False
        section = None

    name_re = re.compile(r'^name:\s*"([^"]+)"$')
    value_re = re.compile(r"^(x|y|z|w):\s*(-?[0-9.eE+\-]+)$")
    for raw in text.splitlines():
        line = raw.strip()
        match = name_re.match(line)
        if match:
            flush()
            name = match.group(1)
            continue
        if name is None:
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


def _read_topic(topic: str, timeout: float = 15.0):
    try:
        proc = subprocess.run(
            ["gz", "topic", "-e", "-n", "1", "-t", topic],
            capture_output=True, text=True, timeout=timeout, check=False,
        )
    except Exception as exc:
        return {}, f"{topic}: {exc!r}"
    if proc.returncode != 0 or not proc.stdout.strip():
        detail = proc.stderr.strip() or "no pose data"
        return {}, f"{topic}: rc={proc.returncode}: {detail}"
    return _parse_pose_text(proc.stdout), ""


def read_calibration_truth():
    dynamic, dynamic_error = _read_topic(POSE_TOPICS[0])
    full, full_error = _read_topic(POSE_TOPICS[1])
    found = {}
    sources = {}
    all_names = sorted(set(dynamic) | set(full))
    for name in all_names:
        if not CAL_NAME.match(name):
            continue
        if name in dynamic:
            found[name] = dynamic[name]
            sources[name] = POSE_TOPICS[0]
        else:
            found[name] = full[name]
            sources[name] = POSE_TOPICS[1]
    errors = [e for e in (dynamic_error, full_error) if e]
    return found, sources, errors


def _validate_truth_set(poses) -> None:
    expected = {f"cal_{i:02d}" for i in range(1, EXPECTED_COUNT + 1)}
    ids = {"_".join(name.split("_")[:2]) for name in poses}
    if len(poses) != EXPECTED_COUNT or ids != expected:
        raise CollectionError(
            f"need exactly 12 cal_* entities; found {sorted(poses)}"
        )


def _hungarian(cost: np.ndarray) -> List[int]:
    """Dependency-free O(n^3) Hungarian assignment; returns column per row."""
    cost = np.asarray(cost, dtype=np.float64)
    if cost.ndim != 2 or cost.shape[0] != cost.shape[1]:
        raise CollectionError(f"assignment matrix must be square, got {cost.shape}")
    n = cost.shape[0]
    u = np.zeros(n + 1)
    v = np.zeros(n + 1)
    p = np.zeros(n + 1, dtype=int)
    way = np.zeros(n + 1, dtype=int)
    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = np.full(n + 1, np.inf)
        used = np.zeros(n + 1, dtype=bool)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = np.inf
            j1 = 0
            for j in range(1, n + 1):
                if used[j]:
                    continue
                cur = cost[i0 - 1, j - 1] - u[i0] - v[j]
                if cur < minv[j]:
                    minv[j] = cur
                    way[j] = j0
                if minv[j] < delta:
                    delta = minv[j]
                    j1 = j
            if not np.isfinite(delta):
                raise CollectionError("assignment has no finite solution")
            for j in range(n + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break
    assignment = [-1] * n
    for j in range(1, n + 1):
        if p[j] > 0:
            assignment[p[j] - 1] = j - 1
    if any(j < 0 for j in assignment):
        raise CollectionError("Hungarian assignment was incomplete")
    return assignment


class OpticalCollector(Node):
    def __init__(self, sync_slop: float):
        super().__init__("collect_extrinsic_calibration")
        self.sync_slop_ns = int(sync_slop * 1e9)
        self.K: Optional[np.ndarray] = None
        self.rgb: Optional[Image] = None
        self.depth: Optional[Image] = None
        self.last_pair: Optional[Tuple[int, int]] = None
        self.create_subscription(
            CameraInfo, f"{CAMERA_NS}/camera_info", self._on_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, f"{CAMERA_NS}/image", self._on_rgb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, f"{CAMERA_NS}/depth_image", self._on_depth,
            qos_profile_sensor_data,
        )

    def _on_info(self, msg: CameraInfo) -> None:
        self.K = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)

    def _on_rgb(self, msg: Image) -> None:
        self.rgb = msg

    def _on_depth(self, msg: Image) -> None:
        self.depth = msg

    def next_observation(self):
        if self.K is None or self.rgb is None or self.depth is None:
            return None
        rgb_stamp = _stamp_ns(self.rgb)
        depth_stamp = _stamp_ns(self.depth)
        pair = (rgb_stamp, depth_stamp)
        if pair == self.last_pair:
            return None
        if abs(rgb_stamp - depth_stamp) > self.sync_slop_ns:
            return None
        self.last_pair = pair
        try:
            rgb = np.frombuffer(self.rgb.data, dtype=np.uint8).reshape(
                self.rgb.height, self.rgb.width, 3,
            )
            depth = np.frombuffer(self.depth.data, dtype=np.float32).reshape(
                self.depth.height, self.depth.width,
            )
        except ValueError:
            return {"error": "image buffer shape mismatch"}

        hsv = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2HSV)
        h, s, value = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        mask = ((s >= SAT_MIN) & (value >= VAL_MIN)).astype(np.uint8)
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        detections = []
        for component in range(1, count):
            area = int(stats[component, cv2.CC_STAT_AREA])
            if not MIN_AREA <= area <= MAX_AREA:
                continue
            blob = labels == component
            colour = _classify_colour(float(np.median(h[blob])))
            if colour is None:
                continue
            depths = depth[blob]
            depths = depths[np.isfinite(depths) & (depths > 0.0)]
            if depths.size < 10:
                continue
            d = float(np.median(depths))
            u, v = (float(vv) for vv in centroids[component])
            optical = np.array([
                (u - cx) * d / fx,
                (v - cy) * d / fy,
                d,
            ], dtype=np.float64)
            detections.append({
                "colour": colour,
                "u": u,
                "v": v,
                "depth_m": d,
                "optical": optical,
                "area_px": area,
            })
        return {
            "rgb_stamp_ns": rgb_stamp,
            "depth_stamp_ns": depth_stamp,
            "sync_delta_s": abs(rgb_stamp - depth_stamp) / 1e9,
            "detections": detections,
        }


def _associate(observation, truth_top, truth_colours):
    detections = observation["detections"]
    if len(detections) != EXPECTED_COUNT:
        raise CollectionError(
            f"frame has {len(detections)} accepted blobs, expected 12"
        )
    result = {}
    for colour in ("red", "green", "yellow"):
        names = sorted(n for n, c in truth_colours.items() if c == colour)
        dets = [d for d in detections if d["colour"] == colour]
        if len(names) != 4 or len(dets) != 4:
            raise CollectionError(
                f"{colour}: truth={len(names)} detections={len(dets)}, need 4/4"
            )
        nominal = [NOMINAL_R @ d["optical"] + NOMINAL_T for d in dets]
        cost = np.array([
            [np.linalg.norm(truth_top[name] - point) for point in nominal]
            for name in names
        ], dtype=np.float64)
        assignment = _hungarian(cost)
        for row, column in enumerate(assignment):
            distance = float(cost[row, column])
            if distance > ASSIGN_MAX_M:
                raise CollectionError(
                    f"{names[row]} nominal assignment is {distance*1000:.1f} mm "
                    f"(limit {ASSIGN_MAX_M*1000:.0f} mm)"
                )
            result[names[row]] = (dets[column], distance)
    if len(result) != EXPECTED_COUNT:
        raise CollectionError(f"assignment produced {len(result)}/12 targets")
    return result


def _geometry_ratio(points: np.ndarray) -> Tuple[np.ndarray, float]:
    values = np.linalg.svd(points - np.mean(points, axis=0), compute_uv=False)
    ratio = float(values[-1] / values[0]) if values[0] > 0.0 else 0.0
    return values, ratio


def _pose_json(pose):
    return {
        "position": [float(v) for v in pose["position"]],
        "orientation_xyzw": [float(v) for v in pose["orientation"]],
    }


def _parse_args(argv=None):
    mirror = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Collect optical/world correspondences for B1 calibration"
    )
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument(
        "--target-half-height", type=float,
        default=DEFAULT_TARGET_HALF_HEIGHT,
        help=("target centre-to-top distance in metres; use 0.002 for the "
              "4 mm B1 tiles (default 0.025 preserves box datasets)"),
    )
    parser.add_argument("--sync-slop", type=float, default=0.050)
    parser.add_argument("--stability-window", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=180.0)
    args = parser.parse_args(argv)
    if args.frames < 30:
        parser.error("--frames must be at least 30 (one-shot calibration samples lie)")
    if not 0.0 < args.target_half_height <= 0.100:
        parser.error("--target-half-height must be in (0, 0.100] metres")
    if args.sync_slop <= 0 or args.stability_window <= 0:
        parser.error("timing arguments must be positive")
    if args.output is None:
        args.output = mirror / "logs" / "v2" / "calibration" / f"{args.run_id}.json"
    return args


def collect(args) -> Path:
    print("===== B1 EXTRINSIC DATASET COLLECTION =====")
    print(
        f"run_id={args.run_id} frames_per_target={args.frames} "
        f"target_half_height={args.target_half_height:.6f} m"
    )

    truth_start_stamp = time.time_ns()
    truth_start, source_start, errors_start = read_calibration_truth()
    _validate_truth_set(truth_start)
    stability_start = time.monotonic()
    time.sleep(args.stability_window)
    truth_ready_stamp = time.time_ns()
    truth_ready, source_ready, errors_ready = read_calibration_truth()
    _validate_truth_set(truth_ready)
    elapsed = max(time.monotonic() - stability_start, 1e-6)

    truth_colours = {}
    truth_top = {}
    stability = {}
    for name in sorted(truth_ready):
        match = CAL_NAME.match(name)
        assert match is not None
        truth_colours[name] = match.group(2)
        start = np.asarray(truth_start[name]["position"], dtype=np.float64)
        ready = np.asarray(truth_ready[name]["position"], dtype=np.float64)
        speed = float(np.linalg.norm(ready - start) / elapsed)
        tilt = _upright_deg(truth_ready[name])
        stability[name] = {"pre_speed_mps": speed, "upright_deg": tilt}
        if speed >= STABILITY_MAX_MPS:
            raise CollectionError(
                f"{name} moving at {speed*1000:.2f} mm/s; limit is <1 mm/s"
            )
        if tilt > UPRIGHT_MAX_DEG:
            raise CollectionError(
                f"{name} tilt {tilt:.3f} deg exceeds {UPRIGHT_MAX_DEG} deg"
            )
        truth_top[name] = _top_world(
            truth_ready[name], args.target_half_height,
        )

    rclpy.init()
    node = OpticalCollector(args.sync_slop)
    raw = {name: [] for name in sorted(truth_ready)}
    valid_frames = 0
    rejected_frames = 0
    rejection_examples: List[str] = []
    collection_start = time.monotonic()
    try:
        deadline = collection_start + args.timeout
        while valid_frames < args.frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            observation = node.next_observation()
            if observation is None:
                continue
            if "error" in observation:
                rejected_frames += 1
                if len(rejection_examples) < 10:
                    rejection_examples.append(observation["error"])
                continue
            try:
                assigned = _associate(observation, truth_top, truth_colours)
            except CollectionError as exc:
                rejected_frames += 1
                if len(rejection_examples) < 10:
                    rejection_examples.append(str(exc))
                continue
            for name, (detection, distance) in assigned.items():
                raw[name].append({
                    "rgb_stamp_ns": observation["rgb_stamp_ns"],
                    "depth_stamp_ns": observation["depth_stamp_ns"],
                    "sync_delta_s": observation["sync_delta_s"],
                    "u": detection["u"],
                    "v": detection["v"],
                    "depth_m": detection["depth_m"],
                    "optical_point_m": detection["optical"].tolist(),
                    "blob_area_px": detection["area_px"],
                    "nominal_assignment_distance_m": distance,
                })
            valid_frames += 1
            print(f"accepted synchronized frame {valid_frames}/{args.frames}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if valid_frames != args.frames:
        raise CollectionError(
            f"timed out with {valid_frames}/{args.frames} valid frames; "
            f"rejected={rejected_frames}; examples={rejection_examples}"
        )

    truth_final_stamp = time.time_ns()
    truth_final, source_final, errors_final = read_calibration_truth()
    _validate_truth_set(truth_final)
    collection_elapsed = max(time.monotonic() - collection_start, 1e-6)

    correspondences = []
    entities = {}
    for name in sorted(truth_final):
        ready_position = np.asarray(truth_ready[name]["position"], dtype=np.float64)
        final_position = np.asarray(truth_final[name]["position"], dtype=np.float64)
        during_speed = float(
            np.linalg.norm(final_position - ready_position) / collection_elapsed
        )
        final_tilt = _upright_deg(truth_final[name])
        if during_speed >= STABILITY_MAX_MPS:
            raise CollectionError(
                f"{name} moved {during_speed*1000:.2f} mm/s during capture"
            )
        if final_tilt > UPRIGHT_MAX_DEG:
            raise CollectionError(
                f"{name} final tilt {final_tilt:.3f} deg exceeds gate"
            )
        samples = raw[name]
        if len(samples) != args.frames:
            raise CollectionError(f"{name} has {len(samples)}/{args.frames} samples")
        optical_median = np.median(np.array([
            sample["optical_point_m"] for sample in samples
        ], dtype=np.float64), axis=0)
        world_top = _top_world(truth_final[name], args.target_half_height)
        assignment_distances = [
            sample["nominal_assignment_distance_m"] for sample in samples
        ]
        entities[name] = {
            "colour": truth_colours[name],
            "truth_source_start": source_start[name],
            "truth_source_ready": source_ready[name],
            "truth_source_final": source_final[name],
            "pose_start": _pose_json(truth_start[name]),
            "pose_ready": _pose_json(truth_ready[name]),
            "pose_final": _pose_json(truth_final[name]),
            "world_top_m": world_top.tolist(),
            "optical_median_m": optical_median.tolist(),
            "pre_speed_mps": stability[name]["pre_speed_mps"],
            "capture_speed_mps": during_speed,
            "upright_deg": final_tilt,
            "assignment_distance_median_m": float(np.median(assignment_distances)),
            "assignment_distance_max_m": float(max(assignment_distances)),
            "raw_samples": samples,
        }
        correspondences.append({
            "entity": name,
            "colour": truth_colours[name],
            "optical_median_m": optical_median.tolist(),
            "world_top_m": world_top.tolist(),
        })

    optical_points = np.array([
        row["optical_median_m"] for row in correspondences
    ], dtype=np.float64)
    world_points = np.array([
        row["world_top_m"] for row in correspondences
    ], dtype=np.float64)
    optical_s, optical_ratio = _geometry_ratio(optical_points)
    world_s, world_ratio = _geometry_ratio(world_points)
    conditioning_ratio = min(optical_ratio, world_ratio)
    conditioning_ok = conditioning_ratio >= CONDITIONING_MIN_RATIO

    dataset = {
        "schema_version": 1,
        "kind": "eye_to_hand_extrinsic_correspondences",
        "run_id": args.run_id,
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "frames_per_target": args.frames,
        "camera": {
            "namespace": CAMERA_NS,
            "intrinsic_matrix": node.K.tolist() if node.K is not None else None,
            "rgb_depth_sync_slop_s": args.sync_slop,
        },
        "truth": {
            "primary_topic": POSE_TOPICS[0],
            "fallback_topic": POSE_TOPICS[1],
            "target_half_height_m": args.target_half_height,
            "local_top_offset_m": [0.0, 0.0, args.target_half_height],
            "snapshot_wall_time_ns": {
                "start": truth_start_stamp,
                "ready": truth_ready_stamp,
                "final": truth_final_stamp,
            },
            "topic_warnings": errors_start + errors_ready + errors_final,
        },
        "association": {
            "method": "per_colour_hungarian",
            "nominal_used_for_association_only": True,
            "max_distance_m": ASSIGN_MAX_M,
            "nominal_rotation_matrix": NOMINAL_R.tolist(),
            "nominal_translation_m": NOMINAL_T.tolist(),
        },
        "collection": {
            "valid_frames": valid_frames,
            "rejected_frames": rejected_frames,
            "rejection_examples": rejection_examples,
            "stability_speed_limit_mps": STABILITY_MAX_MPS,
            "upright_limit_deg": UPRIGHT_MAX_DEG,
        },
        "conditioning": {
            "optical_singular_values": optical_s.tolist(),
            "world_singular_values": world_s.tolist(),
            "optical_ratio": optical_ratio,
            "world_ratio": world_ratio,
            "minimum_ratio": conditioning_ratio,
            "required_minimum_ratio": CONDITIONING_MIN_RATIO,
            "passed": conditioning_ok,
        },
        "entities": entities,
        "correspondences": correspondences,
        "summary": {
            "entity_count": len(entities),
            "samples_per_entity": args.frames,
            "all_associated": len(correspondences) == EXPECTED_COUNT,
            "passed": bool(
                len(correspondences) == EXPECTED_COUNT and conditioning_ok
            ),
        },
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dataset, indent=2, sort_keys=True) + "\n")
    print("\n===== B1 COLLECTION SUMMARY =====")
    print(f"entities: {len(entities)}/12; valid frames: {valid_frames}")
    print(f"rejected frames: {rejected_frames}")
    print(
        f"conditioning ratio: {conditioning_ratio:.6f} "
        f"(required >= {CONDITIONING_MIN_RATIO:.3f})"
    )
    print(f"dataset: {args.output}")
    print("===== " + ("PASS" if dataset["summary"]["passed"] else "FAIL") + " =====")
    if not dataset["summary"]["passed"]:
        raise CollectionError("dataset written for diagnosis but conditioning failed")
    return args.output


def main(argv=None) -> int:
    args = _parse_args(argv)
    try:
        collect(args)
        return 0
    except KeyboardInterrupt:
        print("\n===== FAIL: interrupted =====")
        return 130
    except Exception as exc:
        print(f"\n===== FAIL: {exc} =====")
        return 1


if __name__ == "__main__":
    sys.exit(main())

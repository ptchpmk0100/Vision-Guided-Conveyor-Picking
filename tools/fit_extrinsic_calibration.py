#!/usr/bin/env python3
"""Fit and gate the B1 world<-optical extrinsic with scale-free Kabsch.

After collecting at least two datasets from fresh calibration launches:

  python3 /home/pmkhoi/ros_test_ws_mirror/tools/fit_extrinsic_calibration.py \
    logs/v2/calibration/b1_run_01.json \
    logs/v2/calibration/b1_run_02.json

The gated candidate is written to
``logs/v2/calibration/overhead_camera_extrinsic.candidate.yaml``.  It is deliberately
not a production configuration file.  The ROS/Gazebo-free self-check is:

  python3 /home/pmkhoi/ros_test_ws_mirror/tools/fit_extrinsic_calibration.py --self-test

The transform convention is p_world = R_world_optical @ p_optical + t_world_optical.
No scale is fitted; a similarity scale is calculated only as a diagnostic gate.
"""
import argparse
import datetime as dt
import json
import math
from pathlib import Path
import sys
from typing import Dict, List, Sequence, Tuple

import numpy as np


NOMINAL_R = np.array([
    [0.0, -1.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
], dtype=np.float64)
NOMINAL_T = np.array([0.4, 0.0, 1.2], dtype=np.float64)

EXPECTED_PER_DATASET = 12
GEOMETRY_MIN_RATIO = 0.01
RMS_MAX_M = 0.002
RESIDUAL_MAX_M = 0.004
SCALE_MIN = 0.998
SCALE_MAX = 1.002
ORTHONORMALITY_MAX = 1e-6
DETERMINANT_TOL = 1e-6
NOMINAL_ROTATION_MAX_DEG = 2.0
NOMINAL_TRANSLATION_MAX_M = 0.020
REPEAT_ROTATION_MAX_DEG = 0.15
REPEAT_TRANSLATION_MAX_M = 0.002
REPEAT_CROSS_RMS_MAX_M = 0.003


class FitError(RuntimeError):
    pass


def _as_points(value, label: str) -> np.ndarray:
    points = np.asarray(value, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] < 3:
        raise FitError(f"{label} must be an Nx3 array with N>=3, got {points.shape}")
    if not np.all(np.isfinite(points)):
        raise FitError(f"{label} contains a non-finite value")
    return points


def _geometry(points: np.ndarray) -> Tuple[np.ndarray, float]:
    singular_values = np.linalg.svd(
        points - np.mean(points, axis=0), compute_uv=False,
    )
    ratio = (
        float(singular_values[-1] / singular_values[0])
        if singular_values[0] > np.finfo(float).eps else 0.0
    )
    return singular_values, ratio


def _rotation_difference_deg(a: np.ndarray, b: np.ndarray) -> float:
    relative = a @ b.T
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def _kabsch(optical, world) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return proper R, t, and covariance singular values; never fit scale."""
    optical = _as_points(optical, "optical points")
    world = _as_points(world, "world points")
    if optical.shape != world.shape:
        raise FitError(
            f"optical/world shapes differ: {optical.shape} vs {world.shape}"
        )
    optical_mean = np.mean(optical, axis=0)
    world_mean = np.mean(world, axis=0)
    source = optical - optical_mean
    target = world - world_mean
    covariance = source.T @ target
    u, singular_values, vt = np.linalg.svd(covariance)
    correction = np.eye(3)
    correction[-1, -1] = 1.0 if np.linalg.det(vt.T @ u.T) >= 0.0 else -1.0
    rotation = vt.T @ correction @ u.T
    translation = world_mean - rotation @ optical_mean
    if np.linalg.det(rotation) < 0.0:
        raise FitError("internal error: Kabsch returned an improper rotation")
    return rotation, translation, singular_values


def _diagnostic_scale(optical: np.ndarray, world: np.ndarray,
                      rotation: np.ndarray) -> float:
    source = optical - np.mean(optical, axis=0)
    target = world - np.mean(world, axis=0)
    denominator = float(np.sum(source * source))
    if denominator <= np.finfo(float).eps:
        raise FitError("diagnostic scale is undefined for zero-spread input")
    rotated = (rotation @ source.T).T
    return float(np.sum(target * rotated) / denominator)


def _metrics(optical: np.ndarray, world: np.ndarray,
             rotation: np.ndarray, translation: np.ndarray) -> Dict[str, object]:
    predicted = (rotation @ optical.T).T + translation
    residual_vectors = predicted - world
    residuals = np.linalg.norm(residual_vectors, axis=1)
    optical_s, optical_ratio = _geometry(optical)
    world_s, world_ratio = _geometry(world)
    return {
        "rms_residual_m": float(np.sqrt(np.mean(residuals ** 2))),
        "max_residual_m": float(np.max(residuals)),
        "residuals_m": residuals,
        "residual_vectors_m": residual_vectors,
        "diagnostic_scale": _diagnostic_scale(optical, world, rotation),
        "orthonormality_error": float(np.linalg.norm(
            rotation.T @ rotation - np.eye(3), ord="fro",
        )),
        "determinant": float(np.linalg.det(rotation)),
        "nominal_rotation_error_deg": _rotation_difference_deg(
            rotation, NOMINAL_R,
        ),
        "nominal_translation_error_m": float(np.linalg.norm(
            translation - NOMINAL_T,
        )),
        "optical_singular_values": optical_s,
        "world_singular_values": world_s,
        "optical_geometry_ratio": optical_ratio,
        "world_geometry_ratio": world_ratio,
        "geometry_ratio": min(optical_ratio, world_ratio),
    }


def _gate_metrics(metrics: Dict[str, object], label: str,
                  gate_residuals: bool = True,
                  gate_nominal: bool = True) -> None:
    failures = []
    if float(metrics["geometry_ratio"]) < GEOMETRY_MIN_RATIO:
        failures.append(
            f"geometry ratio {metrics['geometry_ratio']:.6g} < {GEOMETRY_MIN_RATIO}"
        )
    if gate_residuals and float(metrics["rms_residual_m"]) > RMS_MAX_M:
        failures.append(
            f"RMS {metrics['rms_residual_m']*1000:.3f} mm > {RMS_MAX_M*1000:.1f} mm"
        )
    if gate_residuals and float(metrics["max_residual_m"]) > RESIDUAL_MAX_M:
        failures.append(
            f"max residual {metrics['max_residual_m']*1000:.3f} mm > "
            f"{RESIDUAL_MAX_M*1000:.1f} mm"
        )
    scale = float(metrics["diagnostic_scale"])
    if not SCALE_MIN <= scale <= SCALE_MAX:
        failures.append(
            f"diagnostic scale {scale:.9f} outside [{SCALE_MIN}, {SCALE_MAX}]"
        )
    if float(metrics["orthonormality_error"]) > ORTHONORMALITY_MAX:
        failures.append(
            f"orthonormality {metrics['orthonormality_error']:.3e} > "
            f"{ORTHONORMALITY_MAX:.1e}"
        )
    if abs(float(metrics["determinant"]) - 1.0) > DETERMINANT_TOL:
        failures.append(
            f"det(R) {metrics['determinant']:.12f} is not +1"
        )
    if gate_nominal and (
        float(metrics["nominal_rotation_error_deg"]) > NOMINAL_ROTATION_MAX_DEG
    ):
        failures.append(
            f"nominal rotation error {metrics['nominal_rotation_error_deg']:.4f} deg "
            f"> {NOMINAL_ROTATION_MAX_DEG} deg"
        )
    if gate_nominal and (
        float(metrics["nominal_translation_error_m"]) > NOMINAL_TRANSLATION_MAX_M
    ):
        failures.append(
            f"nominal translation error "
            f"{metrics['nominal_translation_error_m']*1000:.3f} mm > "
            f"{NOMINAL_TRANSLATION_MAX_M*1000:.1f} mm"
        )
    if failures:
        raise FitError(f"{label} failed gates: " + "; ".join(failures))


def fit_gated(optical, world, label: str = "fit",
              gate_nominal: bool = True):
    optical = _as_points(optical, f"{label} optical points")
    world = _as_points(world, f"{label} world points")
    optical_s, optical_ratio = _geometry(optical)
    world_s, world_ratio = _geometry(world)
    ratio = min(optical_ratio, world_ratio)
    if ratio < GEOMETRY_MIN_RATIO:
        raise FitError(
            f"{label} is collinear/coplanar or ill-conditioned: geometry ratio "
            f"{ratio:.6g} < {GEOMETRY_MIN_RATIO}; optical S={optical_s.tolist()}, "
            f"world S={world_s.tolist()}"
        )
    rotation, translation, _ = _kabsch(optical, world)
    metrics = _metrics(optical, world, rotation, translation)
    _gate_metrics(metrics, label, gate_nominal=gate_nominal)
    return rotation, translation, metrics


def _load_dataset(path: Path):
    try:
        data = json.loads(path.read_text())
    except Exception as exc:
        raise FitError(f"cannot read {path}: {exc}") from exc
    if data.get("schema_version") != 1:
        raise FitError(f"{path}: unsupported schema_version")
    if data.get("kind") != "eye_to_hand_extrinsic_correspondences":
        raise FitError(f"{path}: wrong dataset kind")
    if not data.get("summary", {}).get("passed", False):
        raise FitError(f"{path}: collector did not mark dataset PASS")
    rows = data.get("correspondences", [])
    if len(rows) != EXPECTED_PER_DATASET:
        raise FitError(
            f"{path}: need exactly {EXPECTED_PER_DATASET} correspondences, "
            f"found {len(rows)}"
        )
    names = [row.get("entity") for row in rows]
    if len(set(names)) != EXPECTED_PER_DATASET or any(
        not isinstance(name, str) or not name.startswith("cal_") for name in names
    ):
        raise FitError(f"{path}: entity IDs are missing or duplicated")
    ordered = sorted(rows, key=lambda row: row["entity"])
    optical = _as_points(
        [row["optical_median_m"] for row in ordered], f"{path} optical",
    )
    world = _as_points(
        [row["world_top_m"] for row in ordered], f"{path} world",
    )
    run_id = str(data.get("run_id") or path.stem)
    return {
        "path": path,
        "run_id": run_id,
        "entities": [row["entity"] for row in ordered],
        "optical": optical,
        "world": world,
    }


def _cross_rms(source_fit, target_dataset) -> float:
    predicted = (
        source_fit["rotation"] @ target_dataset["optical"].T
    ).T + source_fit["translation"]
    errors = np.linalg.norm(predicted - target_dataset["world"], axis=1)
    return float(np.sqrt(np.mean(errors ** 2)))


def _repeatability(run_fits: List[Dict[str, object]]) -> List[Dict[str, object]]:
    comparisons = []
    failures = []
    for i in range(len(run_fits)):
        for j in range(i + 1, len(run_fits)):
            first, second = run_fits[i], run_fits[j]
            rotation_deg = _rotation_difference_deg(
                first["rotation"], second["rotation"],
            )
            translation_m = float(np.linalg.norm(
                first["translation"] - second["translation"],
            ))
            first_on_second = _cross_rms(first, second)
            second_on_first = _cross_rms(second, first)
            row = {
                "runs": [first["run_id"], second["run_id"]],
                "rotation_difference_deg": rotation_deg,
                "translation_difference_m": translation_m,
                "first_fit_on_second_rms_m": first_on_second,
                "second_fit_on_first_rms_m": second_on_first,
            }
            comparisons.append(row)
            if rotation_deg > REPEAT_ROTATION_MAX_DEG:
                failures.append(
                    f"{first['run_id']} vs {second['run_id']} rotation "
                    f"{rotation_deg:.4f} deg > {REPEAT_ROTATION_MAX_DEG} deg"
                )
            if translation_m > REPEAT_TRANSLATION_MAX_M:
                failures.append(
                    f"{first['run_id']} vs {second['run_id']} translation "
                    f"{translation_m*1000:.3f} mm > "
                    f"{REPEAT_TRANSLATION_MAX_M*1000:.1f} mm"
                )
            if max(first_on_second, second_on_first) > REPEAT_CROSS_RMS_MAX_M:
                failures.append(
                    f"{first['run_id']} vs {second['run_id']} cross-run RMS "
                    f"{max(first_on_second, second_on_first)*1000:.3f} mm > "
                    f"{REPEAT_CROSS_RMS_MAX_M*1000:.1f} mm"
                )
    if failures:
        raise FitError("repeatability failed: " + "; ".join(failures))
    return comparisons


def _fmt(value: float) -> str:
    return f"{float(value):.12g}"


def _yaml_quote(value: str) -> str:
    return json.dumps(str(value))


def _candidate_yaml(rotation: np.ndarray, translation: np.ndarray,
                    metrics: Dict[str, object], datasets, repeatability) -> str:
    lines = [
        "# B1 candidate only; Stage 3 must explicitly promote/integrate this file.",
        "schema_version: 1",
        "transform:",
        "  parent_frame: \"world\"",
        "  child_frame: \"camera_optical_link\"",
        "  convention: \"p_world = R_world_optical * p_optical + t_world_optical\"",
        "  units: \"m\"",
        "  rotation_matrix:",
    ]
    for row in rotation:
        lines.append("    - [" + ", ".join(_fmt(v) for v in row) + "]")
    lines.extend([
        "  translation: [" + ", ".join(_fmt(v) for v in translation) + "]",
        "calibration:",
        "  method: \"Kabsch SVD, rigid transform without scale\"",
        f"  generated_utc: {_yaml_quote(dt.datetime.now(dt.timezone.utc).isoformat())}",
        f"  dataset_count: {len(datasets)}",
        f"  correspondence_count: {sum(len(d['optical']) for d in datasets)}",
        "  run_ids:",
    ])
    for dataset in datasets:
        lines.append(f"    - {_yaml_quote(dataset['run_id'])}")
    lines.extend([
        f"  rms_residual_m: {_fmt(metrics['rms_residual_m'])}",
        f"  max_residual_m: {_fmt(metrics['max_residual_m'])}",
        f"  diagnostic_unconstrained_scale: {_fmt(metrics['diagnostic_scale'])}",
        f"  orthonormality_error_fro: {_fmt(metrics['orthonormality_error'])}",
        f"  determinant: {_fmt(metrics['determinant'])}",
        f"  geometry_singular_value_ratio: {_fmt(metrics['geometry_ratio'])}",
        f"  nominal_rotation_error_deg: {_fmt(metrics['nominal_rotation_error_deg'])}",
        f"  nominal_translation_error_m: {_fmt(metrics['nominal_translation_error_m'])}",
        f"  repeatability_evaluated: {'true' if len(datasets) >= 2 else 'false'}",
        f"  repeatability_pair_count: {len(repeatability)}",
        "gates:",
        "  passed: true",
        f"  rms_residual_max_m: {_fmt(RMS_MAX_M)}",
        f"  max_residual_max_m: {_fmt(RESIDUAL_MAX_M)}",
        f"  scale_range: [{_fmt(SCALE_MIN)}, {_fmt(SCALE_MAX)}]",
        f"  geometry_ratio_min: {_fmt(GEOMETRY_MIN_RATIO)}",
        f"  nominal_rotation_max_deg: {_fmt(NOMINAL_ROTATION_MAX_DEG)}",
        f"  nominal_translation_max_m: {_fmt(NOMINAL_TRANSLATION_MAX_M)}",
        f"  repeat_rotation_max_deg: {_fmt(REPEAT_ROTATION_MAX_DEG)}",
        f"  repeat_translation_max_m: {_fmt(REPEAT_TRANSLATION_MAX_M)}",
        f"  repeat_cross_rms_max_m: {_fmt(REPEAT_CROSS_RMS_MAX_M)}",
    ])
    return "\n".join(lines) + "\n"


def _print_fit(label: str, rotation: np.ndarray, translation: np.ndarray,
               metrics: Dict[str, object]) -> None:
    print(f"\n[{label}]")
    print("R_world_optical=")
    for row in rotation:
        print("  " + " ".join(f"{v:+.9f}" for v in row))
    print("t_world_optical=" + " ".join(f"{v:+.9f}" for v in translation))
    print(
        f"RMS={metrics['rms_residual_m']*1000:.3f} mm  "
        f"max={metrics['max_residual_m']*1000:.3f} mm  "
        f"scale={metrics['diagnostic_scale']:.9f}  "
        f"geometry={metrics['geometry_ratio']:.6f}"
    )
    print(
        f"det={metrics['determinant']:.12f}  "
        f"orth={metrics['orthonormality_error']:.3e}  "
        f"nominal dR={metrics['nominal_rotation_error_deg']:.5f} deg  "
        f"dt={metrics['nominal_translation_error_m']*1000:.3f} mm"
    )


def solve(paths: Sequence[Path], output: Path) -> Path:
    if not paths:
        raise FitError("provide at least one dataset (two fresh launches are required for acceptance)")
    datasets = [_load_dataset(path) for path in paths]
    run_ids = [dataset["run_id"] for dataset in datasets]
    if len(set(run_ids)) != len(run_ids):
        raise FitError(f"run IDs must be unique, got {run_ids}")

    run_fits = []
    for dataset in datasets:
        rotation, translation, metrics = fit_gated(
            dataset["optical"], dataset["world"], dataset["run_id"],
        )
        fit = dict(dataset)
        fit.update({
            "rotation": rotation,
            "translation": translation,
            "metrics": metrics,
        })
        run_fits.append(fit)
        _print_fit(dataset["run_id"], rotation, translation, metrics)

    if len(run_fits) < 2:
        print(
            "WARNING: only one dataset supplied; fit gates can pass, but fresh-launch "
            "repeatability is NOT EVALUATED. Use at least two datasets for acceptance."
        )
        repeatability = []
    else:
        repeatability = _repeatability(run_fits)
    for row in repeatability:
        print(
            f"repeat {row['runs'][0]} vs {row['runs'][1]}: "
            f"dR={row['rotation_difference_deg']:.5f} deg  "
            f"dt={row['translation_difference_m']*1000:.3f} mm  "
            f"cross RMS={max(row['first_fit_on_second_rms_m'], row['second_fit_on_first_rms_m'])*1000:.3f} mm"
        )

    pooled_optical = np.vstack([dataset["optical"] for dataset in datasets])
    pooled_world = np.vstack([dataset["world"] for dataset in datasets])
    rotation, translation, metrics = fit_gated(
        pooled_optical, pooled_world, "pooled fit",
    )
    _print_fit("pooled fit", rotation, translation, metrics)

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(_candidate_yaml(
        rotation, translation, metrics, datasets, repeatability,
    ))
    print(f"\n===== PASS: candidate written to {output} =====")
    return output


def _axis_angle(axis: Sequence[float], angle_deg: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=np.float64)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    angle = math.radians(angle_deg)
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def self_test() -> None:
    print("===== B1 KABSCH SELF-TEST =====")
    optical = np.array([
        [-0.18, -0.10, 0.72],
        [0.14, -0.09, 0.79],
        [-0.11, 0.16, 0.86],
        [0.15, 0.13, 0.94],
        [0.02, -0.03, 1.08],
        [-0.20, 0.19, 1.16],
        [0.19, -0.18, 1.25],
        [0.06, 0.21, 1.31],
    ], dtype=np.float64)
    expected_rotation = _axis_angle([0.3, -0.7, 0.2], 23.0)
    expected_translation = np.array([0.41, -0.08, 1.17])
    world = (expected_rotation @ optical.T).T + expected_translation
    rotation, translation, _ = _kabsch(optical, world)
    if _rotation_difference_deg(rotation, expected_rotation) > 1e-6:
        raise FitError("self-test did not recover the synthetic rotation")
    if np.linalg.norm(translation - expected_translation) > 1e-10:
        raise FitError("self-test did not recover the synthetic translation")
    if np.linalg.det(rotation) < 0.0:
        raise FitError("synthetic proper fit returned det(R)<0")
    print("PASS: recovered nontrivial synthetic R,t without scale")

    line = np.array([[v, 2.0 * v, -0.5 * v] for v in np.linspace(-1, 1, 8)])
    line_world = (expected_rotation @ line.T).T + expected_translation
    try:
        fit_gated(line, line_world, "collinear self-test", gate_nominal=False)
    except FitError as exc:
        if "ill-conditioned" not in str(exc):
            raise
    else:
        raise FitError("self-test accepted collinear input")
    print("PASS: rejected collinear input")

    reflected = world.copy()
    reflected[:, 0] *= -1.0
    reflected_rotation, _, _ = _kabsch(optical, reflected)
    reflected_det = float(np.linalg.det(reflected_rotation))
    if reflected_det < 1.0 - DETERMINANT_TOL:
        raise FitError(
            f"reflection self-test produced det(R)={reflected_det:.12f}"
        )
    print(f"PASS: reflection correction retained det(R)={reflected_det:.12f}")
    print("===== SELF-TEST PASS =====")


def _parse_args(argv=None):
    mirror = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Fit a gated rigid world<-optical calibration",
    )
    parser.add_argument("datasets", nargs="*", type=Path)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument(
        "--output", type=Path,
        default=mirror / "logs" / "v2" / "calibration" /
        "overhead_camera_extrinsic.candidate.yaml",
    )
    args = parser.parse_args(argv)
    if args.self_test and args.datasets:
        parser.error("--self-test does not accept dataset paths")
    if not args.self_test and not args.datasets:
        parser.error("provide dataset JSON paths or use --self-test")
    return args


def main(argv=None) -> int:
    args = _parse_args(argv)
    try:
        if args.self_test:
            self_test()
        else:
            solve(args.datasets, args.output)
        return 0
    except Exception as exc:
        print(f"===== FAIL: {exc} =====")
        return 1


if __name__ == "__main__":
    sys.exit(main())

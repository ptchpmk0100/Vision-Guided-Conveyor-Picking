"""Load the calibrated overhead-camera extrinsic (p_world = R @ p_optical + t).

The YAML is written by tools/fit_extrinsic_calibration.py (Kabsch SVD hand-eye fit) and
read by every node that turns camera pixels into world coordinates.
"""
import numpy as np
import yaml


def load_extrinsic(extrinsic_file):
    """Load and strictly validate p_world = R @ p_optical + t."""
    try:
        with open(extrinsic_file, "r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        transform = document["transform"]
        parent_frame = transform["parent_frame"]
        child_frame = transform["child_frame"]
        rotation = np.asarray(transform["rotation_matrix"], dtype=np.float64)
        translation = np.asarray(transform["translation"], dtype=np.float64)
    except Exception as exc:
        raise RuntimeError(
            f"invalid extrinsic file {extrinsic_file!r}: {exc}"
        ) from exc

    if parent_frame != "world" or child_frame != "camera_optical_link":
        raise RuntimeError(
            f"invalid extrinsic frames in {extrinsic_file!r}: expected "
            "world <- camera_optical_link, got "
            f"{parent_frame!r} <- {child_frame!r}"
        )
    if rotation.shape != (3, 3):
        raise RuntimeError(
            f"invalid rotation shape in {extrinsic_file!r}: "
            f"expected (3, 3), got {rotation.shape}"
        )
    if translation.shape != (3,):
        raise RuntimeError(
            f"invalid translation shape in {extrinsic_file!r}: "
            f"expected (3,), got {translation.shape}"
        )
    if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation)):
        raise RuntimeError(f"non-finite extrinsic value in {extrinsic_file!r}")

    orthonormality_error = float(np.linalg.norm(
        rotation.T @ rotation - np.eye(3), ord="fro",
    ))
    determinant = float(np.linalg.det(rotation))
    if orthonormality_error > 1e-6:
        raise RuntimeError(
            f"non-orthonormal rotation in {extrinsic_file!r}: "
            f"||R^T R-I||_F={orthonormality_error:.3e} > 1e-6"
        )
    if abs(determinant - 1.0) > 1e-6:
        raise RuntimeError(
            f"improper rotation in {extrinsic_file!r}: "
            f"det(R)={determinant:.12g}, expected +1 within 1e-6"
        )
    return rotation, translation

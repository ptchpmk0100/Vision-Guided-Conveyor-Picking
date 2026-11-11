"""Depth-only top-plane parcel segmentation, independent of ROS and carton colours."""
import cv2
import numpy as np

from twin_moveit_scripts.parcel_layout import DECK_Z, PARCEL_SIZE


def detect(depth, intrinsics, rotation, translation, roi=(.505, .615, -.180, .180)):
    return detect_regions(depth, intrinsics, rotation, translation, [roi])[0]


def detect_regions(depth, intrinsics, rotation, translation, regions):
    """Deproject once per frame, then segment each requested world region."""
    rows, cols = np.indices(depth.shape)
    fx, fy = intrinsics[0, 0], intrinsics[1, 1]
    cx, cy = intrinsics[0, 2], intrinsics[1, 2]
    optical = np.stack(((cols - cx) * depth / fx, (rows - cy) * depth / fy, depth))
    world = np.einsum('ij,jhw->ihw', rotation, optical) + translation[:, None, None]
    return [_detect_region(depth, world, roi) for roi in regions]


def _detect_region(depth, world, roi):
    x, y, z = world
    valid = (np.isfinite(depth) & (depth > 0) & (x >= roi[0]) & (x <= roi[1])
             & (y >= roi[2]) & (y <= roi[3])
             & (abs(z - (DECK_Z + PARCEL_SIZE[2])) <= .006))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(valid.astype(np.uint8), 8)
    out = []
    for label in range(1, count):
        if not 200 <= stats[label, cv2.CC_STAT_AREA] <= 5000:
            continue
        blob = labels == label
        xs, ys, zs = x[blob], y[blob], z[blob]
        dx, dy = np.ptp(xs), np.ptp(ys)
        if not (.060 <= dx <= .095 and .035 <= dy <= .065):
            continue
        out.append((float(np.median(xs)), float(np.median(ys)),
                    float(np.median(zs) - PARCEL_SIZE[2] / 2)))
    return sorted(out, key=lambda p: -p[1])

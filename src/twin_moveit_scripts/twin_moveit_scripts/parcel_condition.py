"""Parcel condition (intact / damaged): RGB crop geometry shared by dataset export and the runtime node.

Pure numpy/cv2 (no torch, no ROS) so tools, tests and the ONNX node cut IDENTICAL crops.
The crop is a fixed WORLD-size window around a parcel centre projected through the calibrated camera.
"""
import math

import cv2
import numpy as np

CLASSES = ('intact', 'damaged')
CROP_WINDOW_M = 0.12        # world window around the parcel centre (parcel is 80 x 50 mm)
CROP_SIZE = 64              # stored / model-input crop edge in pixels
MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)


def world_to_pixel(point, intrinsics, rotation, translation):
    """World point -> (u, v, depth). rotation/translation are optical->world (p_w = R p_o + t)."""
    p_opt = np.asarray(rotation).T @ (np.asarray(point, float) - np.asarray(translation, float))
    z = float(p_opt[2])
    if z <= 1e-6:
        return None
    return (float(intrinsics[0, 0] * p_opt[0] / z + intrinsics[0, 2]),
            float(intrinsics[1, 1] * p_opt[1] / z + intrinsics[1, 2]), z)


def parcel_corners(centre, yaw, size=(0.080, 0.050, 0.050)):
    """The 8 world corners of an upright parcel."""
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                dx, dy = sx * size[0] / 2, sy * size[1] / 2
                out.append((centre[0] + c * dx - s * dy, centre[1] + s * dx + c * dy,
                            centre[2] + sz * size[2] / 2))
    return out


def crop_window(centre, intrinsics, rotation, translation, shape):
    """(u0, v0, u1, v1) integer pixel window for `centre`, or None when it leaves the image."""
    pix = world_to_pixel(centre, intrinsics, rotation, translation)
    if pix is None:
        return None
    u, v, z = pix
    half = CROP_WINDOW_M / 2 * intrinsics[0, 0] / z
    u0, v0, u1, v1 = (int(round(u - half)), int(round(v - half)), int(round(u + half)), int(round(v + half)))
    if u0 < 0 or v0 < 0 or u1 > shape[1] or v1 > shape[0]:
        return None
    return u0, v0, u1, v1


def cut_crop(rgb, window, size=CROP_SIZE):
    u0, v0, u1, v1 = window
    return cv2.resize(rgb[v0:v1, u0:u1], (size, size), interpolation=cv2.INTER_AREA
                      if (u1 - u0) > size else cv2.INTER_LINEAR)


def preprocess(crops, size=128):
    """uint8 RGB crops (N,H,W,3) -> float32 NCHW normalised model input (bilinear upsample)."""
    batch = np.stack([cv2.resize(c, (size, size), interpolation=cv2.INTER_LINEAR) for c in crops])
    batch = (batch.astype(np.float32) / 255.0 - MEAN) / STD
    return np.ascontiguousarray(batch.transpose(0, 3, 1, 2))


def softmax(logits):
    e = np.exp(logits - logits.max(axis=1, keepdims=True))
    return e / e.sum(axis=1, keepdims=True)

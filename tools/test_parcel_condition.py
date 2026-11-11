#!/usr/bin/env python3
"""Condition-classifier crop geometry: projection, window size, edges, preprocessing contract."""
import math
from pathlib import Path
import unittest

import numpy as np

from twin_moveit_scripts.extrinsic import load_extrinsic as _load_extrinsic
from twin_moveit_scripts.parcel_condition import (CROP_SIZE, CROP_WINDOW_M, crop_window, cut_crop, parcel_corners,
                                                  preprocess, softmax, world_to_pixel)

ROOT = Path(__file__).resolve().parents[1]
R, T = _load_extrinsic(str(ROOT / 'src/twin_action_demo/config/overhead_camera_extrinsic.yaml'))
FX = 320 / math.tan(math.radians(76) / 2)
K = np.array([[FX, 0, 320.5], [0, FX, 240.5], [0, 0, 1.0]])


class ConditionGeometryTest(unittest.TestCase):
    def test_optical_axis_projects_to_principal_point(self):
        point = T + R @ np.array([0, 0, 0.9])
        u, v, z = world_to_pixel(point, K, R, T)
        self.assertAlmostEqual(u, 320.5, places=6)
        self.assertAlmostEqual(v, 240.5, places=6)
        self.assertAlmostEqual(z, 0.9, places=6)

    def test_point_behind_camera_is_rejected(self):
        self.assertIsNone(world_to_pixel(T - R @ np.array([0, 0, 0.5]), K, R, T))

    def test_window_has_fixed_world_size(self):
        shape = (480, 640, 3)
        for y in (-0.3, 0.0, 0.3):
            centre = (0.56, y, 0.32)
            u0, v0, u1, v1 = crop_window(centre, K, R, T, shape)
            z = world_to_pixel(centre, K, R, T)[2]
            self.assertAlmostEqual(u1 - u0, CROP_WINDOW_M * FX / z, delta=1.5)
            self.assertAlmostEqual(u1 - u0, v1 - v0, delta=1)

    def test_window_leaving_the_image_is_none(self):
        self.assertIsNone(crop_window((0.56, -0.64, 0.32), K, R, T, (480, 640, 3)))

    def test_crop_and_preprocess_contract(self):
        rgb = np.random.default_rng(0).integers(0, 255, (480, 640, 3), dtype=np.uint8)
        crop = cut_crop(rgb, crop_window((0.56, 0.0, 0.32), K, R, T, rgb.shape))
        self.assertEqual(crop.shape, (CROP_SIZE, CROP_SIZE, 3))
        batch = preprocess([crop, crop])
        self.assertEqual(batch.shape, (2, 3, 128, 128))
        self.assertEqual(batch.dtype, np.float32)
        self.assertTrue(np.isfinite(batch).all())
        self.assertAlmostEqual(float(softmax(np.array([[0.0, 0.0]]))[0, 1]), 0.5)

    def test_parcel_corners_span_the_box(self):
        pts = np.array(parcel_corners((0, 0, 0), 0.0))
        self.assertAlmostEqual(np.ptp(pts[:, 0]), 0.08)
        self.assertAlmostEqual(np.ptp(pts[:, 1]), 0.05)
        self.assertAlmostEqual(np.ptp(pts[:, 2]), 0.05)


if __name__ == '__main__':
    unittest.main()

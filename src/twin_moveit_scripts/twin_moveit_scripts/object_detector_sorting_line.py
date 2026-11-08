#!/usr/bin/env python3
"""Stage B: multi-object perception for the RAISED sorting line.

A recalibration of object_detector_multi.py for the belt-height scene (additive -- the
multi detector is untouched and still runs the ground-cell pipeline). Same structural HSV
approach (saturation gate + connected components + per-blob median depth + true pinhole
deprojection), which is why raising the objects needs ONLY new height/ROI/area constants,
not a new method: the deprojection uses real depth, so world XY stays valid at any height
(confirmed against move_to's detector math + Stage 0 ground truth).

What changed vs object_detector_multi, and why (all from the sorting-line geometry):
  OBJ_HALF_HEIGHT  0.015 -> 0.025   (40x40x50 mm box, half = 25 mm)
  EXPECTED_CENTER_Z 0.015 -> 0.320  (deck top 0.295 + box half 0.025)
  ROI_X (0.28,0.64) -> (0.28,0.62)  (staging x=0.56 + all tray centres 0.37-0.45)
  ROI_Y (-0.26,0.26) -> (-0.27,0.27) (trays at |y|=0.18)
  MIN/MAX_AREA 60/1500 -> 200/1600  (40 mm box ~650 px at 0.855 m; rejects the hand dots
                                     AND the big ~5850 px coloured tray-floor panels)
  HUE_CLASSES: blue REMOVED (no blue target; the hand's blue dots must never classify)
  shape: forced "box" (uniform cartons; publish <colour>_box)

Tray-floor colour panels can't phantom: their computed centre is z~0.272 (23 mm below the
z-gate) AND their area ~5850 px is far above MAX_AREA -- rejected twice.

Publishes /detected_objects/array (Detection3DArray) + legacy /detected_objects/box.
Honest failure mode: detect nothing -> publish nothing. Never invent a pose.
"""
import numpy as np
import cv2
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import (
    Detection3D, Detection3DArray, ObjectHypothesisWithPose,
)

CAMERA_NS = "/overhead_camera"
ARRAY_TOPIC = "/detected_objects/array"
LEGACY_TOPIC = "/detected_objects/box"

SAT_MIN = 90
VAL_MIN = 50

# 40 mm box at ~0.855 m fills ~650 px. Band rejects the hand's blue dots (small) and the
# big coloured tray-floor panels (~5850 px).
MIN_AREA = 200
MAX_AREA = 1600

# colour classes (OpenCV hue 0-179). NO blue: no blue target, and the hand's indicator
# dots are blue -- they must never be classified.
HUE_CLASSES = (
    ("red",    lambda h: h <= 10 or h >= 170),
    ("yellow", lambda h: 20 <= h <= 35),
    ("green",  lambda h: 40 <= h <= 85),
)

# geometry of the raised line
OBJ_HALF_HEIGHT = 0.025
EXPECTED_CENTER_Z = 0.320
CENTER_Z_TOL = 0.010        # verified from data in Stage B; tray panels compute to ~0.272

# world-frame ROI: staging row (x=0.56) + all three trays (x 0.37-0.45, |y| up to 0.18)
ROI_X = (0.28, 0.62)
ROI_Y = (-0.27, 0.27)


def _optical_to_world(x_opt, y_opt, z_opt):
    """Fixed overhead camera at world (0.4, 0, 1.2) looking straight down."""
    return 0.4 - y_opt, -x_opt, 1.2 - z_opt


def _load_extrinsic(extrinsic_file):
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


def _classify_colour(hue):
    for name, test in HUE_CLASSES:
        if test(hue):
            return name
    return None


class SortingLineDetector(Node):
    def __init__(self):
        super().__init__("object_detector_sorting_line")
        self._K = None
        self._rgb = None
        self._depth = None
        extrinsic_file = self.declare_parameter("extrinsic_file", "").value
        self._extrinsic = None
        if extrinsic_file != "":
            self._extrinsic = _load_extrinsic(extrinsic_file)
            self.get_logger().info(f"extrinsic source: {extrinsic_file}")

        self.create_subscription(CameraInfo, f"{CAMERA_NS}/camera_info",
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, f"{CAMERA_NS}/image",
                                 self._on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, f"{CAMERA_NS}/depth_image",
                                 self._on_depth, qos_profile_sensor_data)

        self._pub_array = self.create_publisher(Detection3DArray, ARRAY_TOPIC, 10)
        self._pub_legacy = self.create_publisher(PoseStamped, LEGACY_TOPIC, 10)
        self.create_timer(0.1, self._tick)

        self._logged_once = False
        self._last_report_ns = 0
        self.get_logger().info(
            f"object_detector_sorting_line up (belt height z={EXPECTED_CENTER_Z}). "
            f"Watching {CAMERA_NS}; publishing {ARRAY_TOPIC} (+ legacy {LEGACY_TOPIC}).")

    def _on_info(self, msg):
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _on_rgb(self, msg):
        self._rgb = self._to_array(msg, 3, np.uint8)

    def _on_depth(self, msg):
        d = self._to_array(msg, 1, np.float32)
        self._depth = None if d is None else d[:, :, 0]

    @staticmethod
    def _to_array(msg, channels, dtype):
        try:
            return np.frombuffer(msg.data, dtype=dtype).reshape(
                msg.height, msg.width, channels)
        except ValueError:
            return None

    def _tick(self):
        if self._K is None or self._rgb is None or self._depth is None:
            return

        rgb = np.ascontiguousarray(self._rgb)
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        H, S, V = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

        mask = ((S >= SAT_MIN) & (V >= VAL_MIN)).astype(np.uint8)
        n_lab, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)

        fx, fy = self._K[0, 0], self._K[1, 1]
        cx, cy = self._K[0, 2], self._K[1, 2]

        detections, report = [], []
        for i in range(1, n_lab):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if not (MIN_AREA <= area <= MAX_AREA):
                continue
            blob = labels == i

            hue = float(np.median(H[blob]))
            colour = _classify_colour(hue)
            if colour is None:
                continue

            dm = self._depth[blob]
            dm = dm[np.isfinite(dm) & (dm > 0.0)]
            if dm.size < 10:
                continue
            d = float(np.median(dm))

            u, v = float(centroids[i][0]), float(centroids[i][1])
            x_opt = (u - cx) * d / fx
            y_opt = (v - cy) * d / fy
            if self._extrinsic is None:
                wx, wy, wz_top = _optical_to_world(x_opt, y_opt, d)
            else:
                rotation, translation = self._extrinsic
                world_top = rotation @ np.array([x_opt, y_opt, d]) + translation
                wx, wy, wz_top = (float(value) for value in world_top)
            wz = wz_top - OBJ_HALF_HEIGHT

            if abs(wz - EXPECTED_CENTER_Z) > CENTER_Z_TOL:
                continue                                # tray-floor panels land here
            if not (ROI_X[0] <= wx <= ROI_X[1] and ROI_Y[0] <= wy <= ROI_Y[1]):
                continue

            cls = f"{colour}_box"                        # uniform cartons -> always a box
            detections.append({"cls": cls, "colour": colour, "shape": "box",
                               "x": wx, "y": wy, "z": wz, "area": area})
            report.append(f"{cls}@({wx:.3f},{wy:.3f},{wz:.3f}) a={area}")

        stamp = self.get_clock().now().to_msg()
        arr = Detection3DArray()
        arr.header.stamp = stamp
        arr.header.frame_id = "world"
        for k, det in enumerate(detections):
            d3 = Detection3D()
            d3.header = arr.header
            d3.id = str(k)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = det["cls"]
            hyp.hypothesis.score = 1.0
            hyp.pose.pose.position.x = det["x"]
            hyp.pose.pose.position.y = det["y"]
            hyp.pose.pose.position.z = det["z"]
            hyp.pose.pose.orientation.w = 1.0
            d3.results.append(hyp)
            d3.bbox.center.position.x = det["x"]
            d3.bbox.center.position.y = det["y"]
            d3.bbox.center.position.z = det["z"]
            d3.bbox.center.orientation.w = 1.0
            d3.bbox.size.x = d3.bbox.size.y = 0.040
            d3.bbox.size.z = 0.050
            arr.detections.append(d3)
        self._pub_array.publish(arr)

        if detections:
            best = max(detections, key=lambda d: d["area"])
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = "world"
            ps.pose.position.x = best["x"]
            ps.pose.position.y = best["y"]
            ps.pose.position.z = best["z"]
            ps.pose.orientation.w = 1.0
            self._pub_legacy.publish(ps)

        now = self.get_clock().now().nanoseconds
        if not self._logged_once or now - self._last_report_ns > 5_000_000_000:
            self._logged_once = True
            self._last_report_ns = now
            self.get_logger().info(
                f"detections({len(detections)}): " + " | ".join(report)
                if report else "detections(0): nothing matched")


def main():
    rclpy.init()
    node = SortingLineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

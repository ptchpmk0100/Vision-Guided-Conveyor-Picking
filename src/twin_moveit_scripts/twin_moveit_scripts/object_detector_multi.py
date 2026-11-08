#!/usr/bin/env python3
"""A2c: MULTI-OBJECT perception -- detect every object, classify colour + shape, publish a list.

Replaces (additively -- the old node stays) the single-blue-threshold detector, which
could only ever see ONE object and sat 5 units away from false-positives:
S21 measured the Panda hand's blue indicator dots at B-G <= 31 against a B-G >= 36
threshold, so merely adding a light pushed arm pixels into the mask and dragged the
centroid 29 mm (see V2_RESUME.md landmines).

WHY HSV FIXES THAT STRUCTURALLY, not by re-tuning:
  Everything in this cell that is NOT a target object -- the floor, bins, pedestal,
  fencing, cabinet, conveyor, and the white/grey Panda itself -- is LOW SATURATION.
  The target objects are vividly coloured. So a saturation gate separates objects from
  the entire rest of the world by construction, and no amount of extra illumination
  turns grey into a saturated hue. Blob-area limits then reject the hand's small
  indicator dots even though they are genuinely blue.

Pipeline:
  RGB --HSV--> saturation+value gate --> binary object mask
      --connected components--> per-blob: area, bbox, centroid
      --filters--> area in range, hue matches a known class, z plausible
      --per blob--> median depth over ITS OWN mask pixels (never a global centroid,
                    which can land off-object on a non-convex/occluded blob -- S21)
      --deproject + fixed optical->world--> world pose of each object

Shape from the top-down silhouette (no training needed):
    aspect = max(w,h)/min(w,h);  fill = blob_area / (w*h)
    aspect > ASPECT_BLOCK            -> "block"     (elongated)
    else fill >= FILL_CUBE           -> "cube"      (square mask fills its bbox)
    else                             -> "cylinder"  (circular mask fills ~pi/4 = 0.785)

Publishes:
  /detected_objects/array  vision_msgs/Detection3DArray  -- every object, classified
  /detected_objects/box    geometry_msgs/PoseStamped     -- LEGACY single-object topic,
      the largest accepted blob. Kept so the existing, ground-truth-verified
      move_to pick_and_place runs unchanged against this node -- that is what lets this
      rewrite be validated with the existing harness before any sorting logic exists.

Honest failure mode, unchanged from the old node: detect nothing -> publish nothing.
Never invent a pose.
"""
import numpy as np
import cv2

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

# ---- object/background separation (the structural fix) ----
SAT_MIN = 90     # 0-255. Grey floor/bins/fence/cabinet/Panda sit far below this.
VAL_MIN = 50     # reject near-black pixels (the conveyor belt, shadows)

# ---- blob size gate ----
# A 3 cm object seen from 1.2 m fills ~200 px (S21 measured 196 px for the cube).
# Generous band around it; also rejects the hand's small blue indicator dots and any
# large saturated backdrop that sneaks past the saturation gate.
MIN_AREA = 60
MAX_AREA = 1500

# ---- shape thresholds (see module docstring) ----
ASPECT_BLOCK = 1.35
FILL_CUBE = 0.88

# ---- colour classes, in OpenCV hue units (0-179) ----
# Blue is INCLUDED so this node can be validated against the existing verified world,
# which still contains the blue cube. The arm's blue dots are excluded by MIN_AREA.
HUE_CLASSES = (
    ("red",    lambda h: h <= 10 or h >= 170),
    ("yellow", lambda h: 20 <= h <= 35),
    ("green",  lambda h: 40 <= h <= 85),
    ("blue",   lambda h: 100 <= h <= 130),
)

# ---- geometry (the invariant: objects rest on the ground plane) ----
OBJ_HALF_HEIGHT = 0.015    # all A2 objects share a 3 cm grasp dimension
EXPECTED_CENTER_Z = 0.015
# Tightened 0.04 -> 0.010 from MEASURED data: all three real objects report z = 0.015
# dead on, while the flat red floor marker computes to z = -0.003 (its top face is only
# 4 mm thick, so subtracting a 15 mm half-height drives it below the ground plane).
# 0.04 was inherited from the single-cube detector and is far looser than the data
# needs; 0.010 still leaves ~10 mm of margin for a genuine object.
CENTER_Z_TOL = 0.010

# ---- WORKSPACE ROI (world frame) ----
# Caught in review before first run: the cell's red/green detection-zone markers are
# saturated, lie flat on the floor (z ~ 0.012, inside CENTER_Z_TOL) and cover ~400 px,
# so they pass the saturation, hue, area and z gates and would be published as phantom
# "red_block"/"green_block" objects every frame. Real cells solve exactly this with a
# region of interest rather than by repainting the plant, so: only accept detections
# whose WORLD position lies in the work area. This also rejects the teal tray and the
# raised feeder-belt cartons for free. Must span the staging tray AND the bins, since
# objects already dropped in a bin still need to be seen.
# Spans the staging row (x 0.56, |y| <= 0.10) AND all three bins (x 0.37-0.45,
# |y| <= 0.18 + 0.07 wall), so objects already dropped in a bin are still seen.
# y cap set from MEASUREMENT, not from the marker's model pose: the red floor marker is
# nominally at y = 0.30, but the camera only sees part of the strip (119 px of ~400), so
# its visible centroid lands at y = 0.272. A 0.28 cap let it through. 0.26 excludes it
# while leaving the bins (objects land at |y| <= 0.18) comfortably inside.
ROI_X = (0.28, 0.64)
ROI_Y = (-0.26, 0.26)


def _optical_to_world(x_opt, y_opt, z_opt):
    """Fixed camera at world (0.4, 0, 1.2) looking straight down.
    Verified mapping, carried over unchanged from object_detector.py."""
    return 0.4 - y_opt, -x_opt, 1.2 - z_opt


def _classify_colour(hue):
    for name, test in HUE_CLASSES:
        if test(hue):
            return name
    return None


def _classify_shape(area, w, h):
    aspect = max(w, h) / max(1.0, min(w, h))
    fill = area / max(1.0, float(w * h))
    if aspect > ASPECT_BLOCK:
        return "block", aspect, fill
    if fill >= FILL_CUBE:
        return "cube", aspect, fill
    return "cylinder", aspect, fill


class MultiObjectDetector(Node):
    def __init__(self):
        super().__init__("object_detector_multi")
        self._K = None
        self._rgb = None
        self._depth = None

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
            f"object_detector_multi up. Watching {CAMERA_NS}; publishing "
            f"{ARRAY_TOPIC} (+ legacy {LEGACY_TOPIC}).")

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

        # np.frombuffer gives a READ-ONLY view; copy so OpenCV always gets a writable,
        # contiguous buffer (cvtColor can refuse a non-writable input on some builds).
        rgb = np.ascontiguousarray(self._rgb)
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        H, S, V = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

        mask = ((S >= SAT_MIN) & (V >= VAL_MIN)).astype(np.uint8)
        n_lab, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)

        fx, fy = self._K[0, 0], self._K[1, 1]
        cx, cy = self._K[0, 2], self._K[1, 2]

        detections, report = [], []
        for i in range(1, n_lab):                       # 0 is background
            area = int(stats[i, cv2.CC_STAT_AREA])
            if not (MIN_AREA <= area <= MAX_AREA):
                continue
            w = int(stats[i, cv2.CC_STAT_WIDTH])
            h = int(stats[i, cv2.CC_STAT_HEIGHT])
            blob = labels == i

            hue = float(np.median(H[blob]))
            colour = _classify_colour(hue)
            if colour is None:
                continue                                # e.g. the teal tray: not a target

            # depth over THIS blob's own pixels, median -> robust to edge/occluder pixels
            dm = self._depth[blob]
            dm = dm[np.isfinite(dm) & (dm > 0.0)]
            if dm.size < 10:
                continue
            d = float(np.median(dm))

            u, v = float(centroids[i][0]), float(centroids[i][1])
            x_opt = (u - cx) * d / fx
            y_opt = (v - cy) * d / fy
            wx, wy, wz_top = _optical_to_world(x_opt, y_opt, d)
            wz = wz_top - OBJ_HALF_HEIGHT

            if abs(wz - EXPECTED_CENTER_Z) > CENTER_Z_TOL:
                continue                                # not resting on the ground
            if not (ROI_X[0] <= wx <= ROI_X[1] and ROI_Y[0] <= wy <= ROI_Y[1]):
                continue                                # outside the work area (markers, decor)

            shape, aspect, fill = _classify_shape(area, w, h)
            detections.append({
                "cls": f"{colour}_{shape}", "colour": colour, "shape": shape,
                "x": wx, "y": wy, "z": wz, "area": area,
                "aspect": aspect, "fill": fill,
            })
            report.append(f"{colour}_{shape}@({wx:.3f},{wy:.3f},{wz:.3f}) "
                          f"a={area} ar={aspect:.2f} fill={fill:.2f}")

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
            d3.bbox.size.x = d3.bbox.size.y = d3.bbox.size.z = 2 * OBJ_HALF_HEIGHT
            arr.detections.append(d3)
        self._pub_array.publish(arr)

        # LEGACY single-object topic: largest accepted blob, so the existing verified
        # pick_and_place runs unchanged against this node.
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
    node = MultiObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

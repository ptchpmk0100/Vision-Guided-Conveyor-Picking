#!/usr/bin/env python3
"""Stage B ground truth: score object_detector_sorting_line against Gazebo poses.

Reads one /detected_objects/array message and the true box poses (dynamic_pose/info via
check_sorting_line.read_poses), matches detections to truth by nearest XY, and reports:
  * detected count, class-correct count (obj_<colour>_box_1 -> '<colour>_box');
  * XY / Z error per box vs ground truth;
  * phantoms (a detection matching no real box) and misses (a real box not detected).

Usage (sim running, detector up):  python3 tools/check_detections_sorting_line.py
Exit 0 iff 3/3 detected, 3/3 class-correct, 0 phantoms, max XY error <= 10 mm.
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_sorting_line import read_poses  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from vision_msgs.msg import Detection3DArray  # noqa: E402

MATCH_XY = 0.05     # a detection within this of a true box is that box
XY_TOL = 0.010      # accuracy gate
Z_TOL = 0.010


def _truth_boxes():
    out = {}
    for name, (x, y, z) in read_poses().items():
        if re.match(r"^obj_\w+_box_\d+$", name):
            out[name] = (name.split("_")[1], x, y, z)   # colour, x, y, z
    return out


class _Grab(Node):
    def __init__(self):
        super().__init__("check_detections_sorting_line")
        self.msg = None
        self.create_subscription(Detection3DArray, "/detected_objects/array",
                                 self._cb, 10)

    def _cb(self, m):
        self.msg = m


def main():
    truth = _truth_boxes()
    rclpy.init()
    node = _Grab()
    import time
    deadline = time.time() + 15.0
    while time.time() < deadline and node.msg is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    dets = []
    if node.msg:
        for d in node.msg.detections:
            if d.results:
                p = d.bbox.center.position
                dets.append((d.results[0].hypothesis.class_id, p.x, p.y, p.z))
    node.destroy_node()
    rclpy.shutdown()

    print("\n===== SORTING-LINE DETECTOR GROUND TRUTH (Stage B) =====")
    if not truth:
        print("!! no ground-truth boxes found")
        return 1

    used = set()
    correct = 0
    worst_xy = 0.0
    for name in sorted(truth):
        colour, tx, ty, tz = truth[name]
        want_cls = f"{colour}_box"
        best, bestd = None, MATCH_XY
        for j, (cls, x, y, z) in enumerate(dets):
            if j in used:
                continue
            dxy = ((x - tx) ** 2 + (y - ty) ** 2) ** 0.5
            if dxy <= bestd:
                best, bestd = j, dxy
        if best is None:
            print(f"MISSED   {name:18s} want={want_cls}")
            continue
        used.add(best)
        cls, x, y, z = dets[best]
        dxy = ((x - tx) ** 2 + (y - ty) ** 2) ** 0.5
        dz = abs(z - tz)
        worst_xy = max(worst_xy, dxy)
        cls_ok = (cls == want_cls)
        acc_ok = (dxy <= XY_TOL and dz <= Z_TOL)
        if cls_ok and acc_ok:
            correct += 1
        print(f"{'OK' if (cls_ok and acc_ok) else 'BAD':3s}  {name:18s} "
              f"det={cls:10s} want={want_cls:10s} "
              f"xy_err={dxy*1000:5.1f}mm z_err={dz*1000:5.1f}mm")
    phantoms = [dets[j] for j in range(len(dets)) if j not in used]
    for cls, x, y, z in phantoms:
        print(f"PHANTOM  det={cls} @({x:+.3f},{y:+.3f},{z:+.3f})")
    print(f"===== {correct}/{len(truth)} class-correct+accurate; "
          f"{len(dets)} detected; {len(phantoms)} phantom; worst_xy={worst_xy*1000:.1f}mm =====")
    return 0 if (correct == len(truth) == 3 and not phantoms and worst_xy <= XY_TOL) else 1


if __name__ == "__main__":
    sys.exit(main())

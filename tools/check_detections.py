#!/usr/bin/env python3
"""A2d: score the multi-object detector against Gazebo ground truth.

Leg (a) of the three-legged validation, for perception: does the detector see every
object, in the right place, with the right class? Ground truth comes from the
simulator, never from the detector's own logs.

Ground truth: `gz topic -e -n 1 -t /world/empty/pose/info` lists every entity with its
name and pose. Objects are named by convention  obj_<colour>_<shape>_<n>  so the name
*is* the label -- no hand-annotation needed.

Detections: one message from /detected_objects/array (vision_msgs/Detection3DArray),
whose class_id is "<colour>_<shape>".

Matching: each ground-truth object takes its nearest detection within MATCH_RADIUS.
Reports detection rate, class accuracy, XY position error, and any spurious extra
detections (phantoms) -- which is how the floor-marker bug would have shown up.

Usage (with the sim already running):  python3 tools/check_detections.py
Exit code 0 if every object was found AND classified correctly, else 1.
"""
import re
import subprocess
import sys
import math

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray

MATCH_RADIUS = 0.06      # m -- generous; a real mismatch is far larger
POS_TOL = 0.02           # m -- XY error we consider "good"
WAIT_SEC = 15.0


def gz_ground_truth():
    """-> {name: (x, y, z)} for every entity whose name starts with 'obj_'."""
    try:
        out = subprocess.run(
            ["gz", "topic", "-e", "-n", "1", "-t", "/world/empty/pose/info"],
            capture_output=True, text=True, timeout=15).stdout
    except Exception as e:
        print(f"!! could not read gz ground truth: {e}")
        return {}

    truth, name, depth_in_pos = {}, None, False
    pos = {}
    for line in out.splitlines():
        s = line.strip()
        m = re.match(r'name:\s*"([^"]+)"', s)
        if m:
            name, pos, depth_in_pos = m.group(1), {}, False
            continue
        if s.startswith("position"):
            depth_in_pos = True
            continue
        if depth_in_pos:
            m = re.match(r'([xyz]):\s*(-?[0-9.eE+-]+)', s)
            if m:
                pos[m.group(1)] = float(m.group(2))
                if len(pos) == 3:
                    if name and name.startswith("obj_"):
                        truth[name] = (pos["x"], pos["y"], pos["z"])
                    depth_in_pos = False
    return truth


def label_from_name(n):
    """obj_red_cube_1 -> red_cube"""
    parts = n.split("_")
    return f"{parts[1]}_{parts[2]}" if len(parts) >= 3 else "?"


class Scorer(Node):
    def __init__(self):
        super().__init__("check_detections")
        self.msg = None
        self.create_subscription(Detection3DArray, "/detected_objects/array",
                                 self._cb, 10)

    def _cb(self, m):
        if self.msg is None:
            self.msg = m


def main():
    rclpy.init()
    node = Scorer()
    deadline = node.get_clock().now().nanoseconds + int(WAIT_SEC * 1e9)
    while node.msg is None and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    dets = []
    if node.msg is not None:
        for d in node.msg.detections:
            if d.results:
                p = d.bbox.center.position
                dets.append((d.results[0].hypothesis.class_id, p.x, p.y, p.z))
    node.destroy_node()
    rclpy.shutdown()

    truth = gz_ground_truth()
    print("\n===== DETECTION SCORE vs GROUND TRUTH =====")
    if not truth:
        print("!! no ground-truth objects found (names must start with 'obj_')")
        return 1
    if node_msg_missing := (not dets):
        print(f"!! no detections received on /detected_objects/array within {WAIT_SEC}s")

    used, found, correct = set(), 0, 0
    for name in sorted(truth):
        tx, ty, tz = truth[name]
        want = label_from_name(name)
        best, bestd = None, 1e9
        for i, (cls, dx, dy, dz) in enumerate(dets):
            if i in used:
                continue
            dist = math.hypot(dx - tx, dy - ty)
            if dist < bestd:
                best, bestd = i, dist
        if best is not None and bestd <= MATCH_RADIUS:
            used.add(best)
            found += 1
            got = dets[best][0]
            ok = (got == want)
            correct += 1 if ok else 0
            flag = "OK " if ok and bestd <= POS_TOL else ("CLASS" if not ok else "POS")
            print(f"{flag:5s} {name:22s} truth=({tx:+.3f},{ty:+.3f}) "
                  f"want={want:14s} got={got:14s} err={bestd*1000:5.1f}mm")
        else:
            print(f"MISS  {name:22s} truth=({tx:+.3f},{ty:+.3f}) want={want:14s} "
                  f"-- no detection within {MATCH_RADIUS*1000:.0f}mm")

    phantoms = [dets[i] for i in range(len(dets)) if i not in used]
    for cls, x, y, z in phantoms:
        print(f"PHANTOM extra detection {cls} @ ({x:+.3f},{y:+.3f},{z:+.3f})")

    n = len(truth)
    print(f"----- detected {found}/{n} | class-correct {correct}/{n} | "
          f"phantoms {len(phantoms)} -----")
    return 0 if (found == n and correct == n and not phantoms) else 1


if __name__ == "__main__":
    sys.exit(main())

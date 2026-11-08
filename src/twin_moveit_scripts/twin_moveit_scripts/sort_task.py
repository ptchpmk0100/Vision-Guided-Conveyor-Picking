#!/usr/bin/env python3
"""A4/A5: sort every staged object into the bin that matches its class.

  ros2 run twin_moveit_scripts sort_task            # sort until the staging area is clear
  ros2 run twin_moveit_scripts sort_task --max 2    # cap the number of objects moved

Workflow (deliberately ONE-WAY: staging row -> bin, never bin -> anywhere):
  1. read a fresh object list from /detected_objects/array
  2. drop any object already inside a bin footprint -- those are done
  3. take the next pending object, map its COLOUR to a bin, pick it at its detected
     pose and place it in that bin
  4. repeat until nothing is pending (or a guard trips)
  5. park at 'ready'

Why one-way matters (A3): with the bins published as MoveIt collision objects,
descending INTO a bin to pick an object back out is unplannable -- OMPL correctly
refuses. The sort task never needs to, which is exactly why A3 is validated here and
not with the A1 ring-tour test.

Colour -> bin is the mapping; shape is detected and logged but does not route (the
3-bin cell has one bin per colour). Shape routing becomes the S3 benchmark.

Identity tracking (A5): objects are tracked by NEAREST-PREVIOUS pose, not by a rounded
coordinate key. A grasp that nudges an object used to change its (cls, round x, round y)
key, so it looked brand-new and got retried without bound (only LOOP_GUARD stopped it).
Now each physical object is one record; a retry is a deliberate, counted second attempt
(MAX_ATTEMPTS_PER_OBJECT), not an accident of a shifted key.

Honest accounting (A5): every object's outcome is recorded and printed at the end, and --
when TWIN_DB is set -- written to SQLite (runs/trials). Success is judged by GROUND TRUTH
afterwards (tools/check_sort.py / the benchmark harness), never by these log lines.
"""
import math
import os
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from vision_msgs.msg import Detection3DArray

from twin_moveit_scripts.move_to import (
    setup, shutdown, move_to_named, pick, place,
)

DETECT_TOPIC = "/detected_objects/array"

# colour -> (bin name, x, y, z). All three reach-verified 2/2 by tools/reach_probe.py.
BIN_FOR_COLOUR = {
    "red":    ("bin_a", 0.45, 0.18, 0.015),
    "green":  ("bin_b", 0.45, -0.18, 0.015),
    "yellow": ("bin_c", 0.37, 0.00, 0.015),
}
# An object whose centre is within this of a bin centre counts as already sorted.
BIN_RADIUS = 0.075
# Release height above the nominal place z, so the open fingers clear the 0.03 m bin
# walls. Reaching to the bin floor puts the opened fingers (0.08 m span, ~0.10 padded)
# inside a 0.132 m interior -> MoveIt finds finger/wall contact and the retract cannot
# be planned, so place() reports retract_failed on an object it actually delivered.
# 0.035 puts the fingertips ~8 mm above the rim; the object drops the last few cm into
# the bin, which is how bin-placing works in practice.
BIN_RELEASE_CLEARANCE = 0.035
# A detection within this of a tracked object's last pose IS that object (identity).
MATCH_RADIUS = 0.08
# Deliberate retry budget per physical object. Cylinders roll on first contact (the
# grasp-width gate correctly refuses the off-centre grasp) and succeed on the second
# try once they have settled -- so 2 is a MEASURED policy, not thrashing. The proper
# fix (shape-aware/antipodal grasp) is B2; do NOT widen GRASP_WIDTH_TOL to paper over it.
MAX_ATTEMPTS_PER_OBJECT = 2
LOOP_GUARD = 12                  # hard backstop, so a mis-detection cannot spin forever


class _DetectionCache:
    """Latest Detection3DArray, with clear-and-wait freshness (same pattern as the
    pose cache in move_to: guarantee the message arrived DURING this call rather than
    comparing clocks, which mismatch between wall-time and sim-time)."""

    def __init__(self, node, logger):
        self._node = node
        self._logger = logger
        self._latest = None
        node.create_subscription(Detection3DArray, DETECT_TOPIC, self._cb, 10)
        logger.info(f"sort_task: subscribed to {DETECT_TOPIC}")

    def _cb(self, msg):
        self._latest = msg

    def get_fresh(self, wait=3.0):
        exec_ = SingleThreadedExecutor()
        exec_.add_node(self._node)
        try:
            self._latest = None
            deadline = time.time() + wait
            while time.time() < deadline:
                exec_.spin_once(timeout_sec=0.1)
                if self._latest is not None:
                    break
            if self._latest is None:
                self._logger.warn(f"sort_task: no detections within {wait}s")
                return []
            out = []
            for d in self._latest.detections:
                if not d.results:
                    continue
                cls = d.results[0].hypothesis.class_id
                p = d.bbox.center.position
                parts = cls.split("_")
                colour = parts[0]
                shape = parts[1] if len(parts) > 1 else "?"
                out.append({"cls": cls, "colour": colour, "shape": shape,
                            "x": p.x, "y": p.y, "z": p.z})
            return out
        finally:
            exec_.remove_node(self._node)


class _ObjectTracker:
    """Track physical objects by nearest-previous pose so a nudged object is recognised
    as the SAME object (and its deliberate retry counted), not mistaken for a new one."""

    def __init__(self):
        self._recs = []  # dicts: colour, cls, x, y, attempts

    def get(self, o):
        """Return the record for detection o (matching by colour + nearest pose within
        MATCH_RADIUS), creating one the first time this object is seen."""
        best, bestd = None, MATCH_RADIUS
        for r in self._recs:
            if r["colour"] != o["colour"]:
                continue
            d = math.hypot(r["x"] - o["x"], r["y"] - o["y"])
            if d <= bestd:
                best, bestd = r, d
        if best is None:
            best = {"colour": o["colour"], "cls": o["cls"],
                    "x": o["x"], "y": o["y"], "attempts": 0}
            self._recs.append(best)
        return best

    def mark_attempt(self, rec, o):
        rec["attempts"] += 1
        rec["x"], rec["y"] = o["x"], o["y"]   # follow the (possibly nudged) object
        return rec["attempts"]


def _in_a_bin(obj):
    for _, (bname, bx, by, _bz) in BIN_FOR_COLOUR.items():
        if ((obj["x"] - bx) ** 2 + (obj["y"] - by) ** 2) ** 0.5 <= BIN_RADIUS:
            return bname
    return None


def _open_db():
    """Open the benchmark DB iff TWIN_DB is set. Returns (conn, run_id, scenario) or
    (None, None, None). Any failure is swallowed -- logging must never break a sort."""
    if "TWIN_DB" not in os.environ:
        return None, None, None
    try:
        from twin_moveit_scripts import trial_db
        conn = trial_db.connect(os.environ["TWIN_DB"])
        run_id = os.environ.get("TWIN_RUN_ID") or time.strftime("run_%Y%m%d_%H%M%S")
        scenario = os.environ.get("TWIN_SCENARIO", "adhoc")
        launch = os.environ.get("TWIN_LAUNCH", "")
        trial_db.start_run(conn, run_id, scenario, launch)
        return conn, run_id, scenario
    except Exception as e:
        print(f"sort_task: DB disabled ({e!r})")
        return None, None, None


def main():
    argv = sys.argv[1:]
    max_objects = None
    if "--max" in argv:
        try:
            max_objects = int(argv[argv.index("--max") + 1])
        except (IndexError, ValueError):
            pass

    robot, arm, logger, params, twin = setup(node_name="sort_task")
    cache = _DetectionCache(twin, logger)
    tracker = _ObjectTracker()
    outcomes = []
    conn, run_id, _ = _open_db()
    seq = 0
    t_run = time.time()

    try:
        from twin_moveit_scripts import trial_db
    except Exception:
        trial_db = None

    try:
        move_to_named(robot, arm, logger, params, "ready")

        for loop in range(LOOP_GUARD):
            objs = cache.get_fresh()
            pending = []
            for o in objs:
                if _in_a_bin(o) is not None:
                    continue
                rec = tracker.get(o)
                if rec["attempts"] < MAX_ATTEMPTS_PER_OBJECT:
                    pending.append((rec, o))
            logger.info(f"sort_task: loop {loop}: {len(objs)} seen, "
                        f"{len(pending)} pending")
            if not pending:
                logger.info("sort_task: staging area clear")
                break
            if max_objects is not None and len(outcomes) >= max_objects:
                logger.info(f"sort_task: reached --max {max_objects}")
                break

            rec, obj = pending[0]
            attempt = tracker.mark_attempt(rec, obj)
            route = BIN_FOR_COLOUR.get(obj["colour"])
            if route is None:
                logger.warn(f"sort_task: no bin for colour '{obj['colour']}' "
                            f"({obj['cls']}) -- leaving it")
                outcomes.append((obj["cls"], "no_bin", None))
                if conn:
                    trial_db.record_trial(conn, run_id, seq, obj, None, attempt,
                                          False, "no_bin", None, None, 0.0, "no_bin")
                    seq += 1
                continue
            bname, bx, by, bz = route
            logger.info(f"sort_task: {obj['cls']} @ "
                        f"({obj['x']:.3f},{obj['y']:.3f}) -> {bname} "
                        f"(attempt {attempt}/{MAX_ATTEMPTS_PER_OBJECT})")

            t_cycle = time.time()
            pr = pick(robot, arm, logger, params, twin,
                      (obj["x"], obj["y"], obj["z"]))
            if not pr.ok:
                logger.error(f"sort_task: pick failed ({pr.stage}) for {obj['cls']}")
                outcomes.append((obj["cls"], f"pick_{pr.stage}", bname))
                if conn:
                    trial_db.record_trial(
                        conn, run_id, seq, obj, bname, attempt,
                        False, pr.stage, None, None,
                        time.time() - t_cycle, f"pick_{pr.stage}")
                    seq += 1
                move_to_named(robot, arm, logger, params, "ready")
                continue

            pl = place(robot, arm, logger, params, twin, (bx, by, bz),
                       clearance=BIN_RELEASE_CLEARANCE)
            outcome = "placed" if pl.ok else f"place_{pl.stage}"
            outcomes.append((obj["cls"], outcome, bname))
            if conn:
                trial_db.record_trial(
                    conn, run_id, seq, obj, bname, attempt,
                    True, pr.stage, pl.ok, pl.stage,
                    time.time() - t_cycle, outcome)
                seq += 1
            if not pl.ok:
                logger.error(f"sort_task: place failed ({pl.stage}) for {obj['cls']}")
            move_to_named(robot, arm, logger, params, "ready")
    finally:
        try:
            move_to_named(robot, arm, logger, params, "ready")
        except Exception as e:
            logger.warn(f"sort_task: final home raised: {e}")
        ok = sum(1 for _, s, _ in outcomes if s == "placed")
        if conn:
            try:
                trial_db.finish_run(conn, run_id, ok, time.time() - t_run)
                conn.close()
            except Exception as e:
                print(f"sort_task: DB finish failed ({e!r})")
        print("\n===== SORT TASK OUTCOMES =====")
        for cls, status, bname in outcomes:
            print(f"SORT {cls:18s} -> {str(bname):6s}  {status}")
        print(f"===== {ok}/{len(outcomes)} placed (ground truth decides -- "
              f"see tools/check_sort.py) =====")
        sys.stdout.flush()
        shutdown(twin)


if __name__ == "__main__":
    main()

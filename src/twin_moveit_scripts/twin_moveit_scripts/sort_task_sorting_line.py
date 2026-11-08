#!/usr/bin/env python3
"""Stage C/D: sort the belt-height colour boxes into their matching trays.

The raised-scene analogue of sort_task.py (additive -- the ground sort_task is untouched).
Reuses the generic machinery (_DetectionCache, _ObjectTracker) from sort_task; only the
geometry differs:
  * trays at belt height (z=0.320), colour -> tray;
  * grasp-width gate retargeted to the 40 mm box (half 0.020) via pick(expect_half_width=);
  * release clearance 0.035 so the open fingers clear the 0.040 m tray walls;
  * the 3 trays are published as MoveIt collision objects (sorting_line_scene) so the place
    motion is genuinely collision-aware.

Workflow is one-way (deck -> tray), same as the ground sort: detect, choose an unsorted box,
map colour to tray, pick, place, park at ready; repeat until clear. Success is judged by
Gazebo ground truth (tools/check_sorting_line.py --placed), never by these log lines.

  ros2 run twin_moveit_scripts sort_task_sorting_line            # sort all boxes
  ros2 run twin_moveit_scripts sort_task_sorting_line --max 1    # one box (Stage C isolation)
"""
import os
import sys
import time

from twin_moveit_scripts.move_to import (
    setup, shutdown, move_to_named, pick, place,
)
from twin_moveit_scripts.sort_task import (
    _DetectionCache, _ObjectTracker, MAX_ATTEMPTS_PER_OBJECT, LOOP_GUARD,
)

# colour -> (tray name, x, y, z) at belt height. All reach-verified GO (V2_DECISIONS D5).
TRAY_FOR_COLOUR = {
    "red":    ("tray_red", 0.450, 0.180, 0.320),
    "green":  ("tray_green", 0.450, -0.180, 0.320),
    "yellow": ("tray_yellow", 0.370, 0.000, 0.320),
}
TRAY_RADIUS = 0.075          # an object within this of a tray centre is already sorted
TRAY_RELEASE_CLEARANCE = 0.035
EXPECT_HALF_WIDTH = 0.020    # 40 mm box seats the fingers at ~0.020 (not the 0.015 cube)
WIDTH_TOL = 0.006


def _in_a_tray(obj):
    for _, (tname, tx, ty, _tz) in TRAY_FOR_COLOUR.items():
        if ((obj["x"] - tx) ** 2 + (obj["y"] - ty) ** 2) ** 0.5 <= TRAY_RADIUS:
            return tname
    return None


def _open_db():
    if "TWIN_DB" not in os.environ:
        return None, None
    try:
        from twin_moveit_scripts import trial_db
        conn = trial_db.connect(os.environ["TWIN_DB"])
        run_id = os.environ.get("TWIN_RUN_ID") or time.strftime("sline_%Y%m%d_%H%M%S")
        trial_db.start_run(conn, run_id, os.environ.get("TWIN_SCENARIO", "sline"),
                           os.environ.get("TWIN_LAUNCH", "industrial_sorting_line"))
        return conn, run_id
    except Exception as e:
        print(f"sort_task_sorting_line: DB disabled ({e!r})")
        return None, None


def main():
    argv = sys.argv[1:]
    max_objects = None
    if "--max" in argv:
        try:
            max_objects = int(argv[argv.index("--max") + 1])
        except (IndexError, ValueError):
            pass

    robot, arm, logger, params, twin = setup(node_name="sort_task_sorting_line")

    # Stage C: make the place motion collision-aware of the tray walls.
    try:
        from twin_moveit_scripts.sorting_line_scene import add_sorting_line_collision
        add_sorting_line_collision(robot, logger)
    except Exception as e:
        logger.warn(f"sort_task_sorting_line: collision scene skipped: {e!r}")

    cache = _DetectionCache(twin, logger)
    tracker = _ObjectTracker()
    outcomes = []
    conn, run_id = _open_db()
    try:
        from twin_moveit_scripts import trial_db
    except Exception:
        trial_db = None
    seq = 0
    t_run = time.time()

    try:
        move_to_named(robot, arm, logger, params, "ready")

        for loop in range(LOOP_GUARD):
            objs = cache.get_fresh()
            pending = []
            for o in objs:
                if _in_a_tray(o) is not None:
                    continue
                rec = tracker.get(o)
                if rec["attempts"] < MAX_ATTEMPTS_PER_OBJECT:
                    pending.append((rec, o))
            logger.info(f"sort_line: loop {loop}: {len(objs)} seen, {len(pending)} pending")
            if not pending:
                logger.info("sort_line: staging area clear")
                break
            if max_objects is not None and len(outcomes) >= max_objects:
                logger.info(f"sort_line: reached --max {max_objects}")
                break

            rec, obj = pending[0]
            attempt = tracker.mark_attempt(rec, obj)
            route = TRAY_FOR_COLOUR.get(obj["colour"])
            if route is None:
                logger.warn(f"sort_line: no tray for colour '{obj['colour']}' -- leaving it")
                outcomes.append((obj["cls"], "no_tray", None))
                continue
            tname, tx, ty, tz = route
            logger.info(f"sort_line: {obj['cls']} @ ({obj['x']:.3f},{obj['y']:.3f}) -> "
                        f"{tname} (attempt {attempt}/{MAX_ATTEMPTS_PER_OBJECT})")

            t_cycle = time.time()
            pr = pick(robot, arm, logger, params, twin,
                      (obj["x"], obj["y"], obj["z"]),
                      expect_half_width=EXPECT_HALF_WIDTH, width_tol=WIDTH_TOL)
            if not pr.ok:
                logger.error(f"sort_line: pick failed ({pr.stage}) for {obj['cls']}")
                outcomes.append((obj["cls"], f"pick_{pr.stage}", tname))
                if conn and trial_db:
                    trial_db.record_trial(conn, run_id, seq, obj, tname, attempt,
                                          False, pr.stage, None, None,
                                          time.time() - t_cycle, f"pick_{pr.stage}")
                    seq += 1
                move_to_named(robot, arm, logger, params, "ready")
                continue

            pl = place(robot, arm, logger, params, twin, (tx, ty, tz),
                       clearance=TRAY_RELEASE_CLEARANCE)
            outcome = "placed" if pl.ok else f"place_{pl.stage}"
            outcomes.append((obj["cls"], outcome, tname))
            if conn and trial_db:
                trial_db.record_trial(conn, run_id, seq, obj, tname, attempt,
                                      True, pr.stage, pl.ok, pl.stage,
                                      time.time() - t_cycle, outcome)
                seq += 1
            if not pl.ok:
                logger.error(f"sort_line: place failed ({pl.stage}) for {obj['cls']}")
            move_to_named(robot, arm, logger, params, "ready")
    finally:
        try:
            move_to_named(robot, arm, logger, params, "ready")
        except Exception as e:
            logger.warn(f"sort_line: final home raised: {e}")
        ok = sum(1 for _, s, _ in outcomes if s == "placed")
        if conn and trial_db:
            try:
                trial_db.finish_run(conn, run_id, ok, time.time() - t_run)
                conn.close()
            except Exception as e:
                print(f"sort_task_sorting_line: DB finish failed ({e!r})")
        print("\n===== SORT LINE OUTCOMES =====")
        for cls, status, tname in outcomes:
            print(f"SORTLINE {cls:14s} -> {str(tname):12s} {status}")
        print(f"===== {ok}/{len(outcomes)} placed (ground truth decides) =====")
        sys.stdout.flush()
        shutdown(twin)


if __name__ == "__main__":
    main()

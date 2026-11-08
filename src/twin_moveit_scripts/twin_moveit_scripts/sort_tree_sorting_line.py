#!/usr/bin/env python3
"""B3 behaviour-tree orchestration for the verified raised sorting line.

The motion, perception, routing, identity tracking, collision scene, and trial database
are the same primitives used by sort_task_sorting_line.  This module changes only the
orchestration policy and makes recovery visible as behaviour-tree ticks.

  ros2 run twin_moveit_scripts sort_tree_sorting_line
  ros2 run twin_moveit_scripts sort_tree_sorting_line --max 1
"""
import sys
import time

from twin_moveit_scripts.behaviour_tree import Leaf, Retry, Sequence, Status
from twin_moveit_scripts.move_to import (
    move_to_named, pick, place, setup, shutdown,
)
from twin_moveit_scripts.sort_task import (
    LOOP_GUARD, MAX_ATTEMPTS_PER_OBJECT, _DetectionCache, _ObjectTracker,
)
from twin_moveit_scripts.sort_task_sorting_line import (
    EXPECT_HALF_WIDTH, TRAY_FOR_COLOUR, TRAY_RELEASE_CLEARANCE, WIDTH_TOL,
    _in_a_tray, _open_db,
)
from twin_moveit_scripts.sorting_line_scene import add_sorting_line_collision


def _record_trial(board, obj, tray, attempt, pick_ok, pick_stage,
                  place_ok, place_stage, elapsed, outcome):
    if not board["conn"] or board["trial_db"] is None:
        return
    try:
        board["trial_db"].record_trial(
            board["conn"], board["run_id"], board["seq"], obj, tray, attempt,
            pick_ok, pick_stage, place_ok, place_stage, elapsed, outcome,
        )
        board["seq"] += 1
    except Exception as exc:
        board["logger"].warn(f"sort_tree: DB trial write failed: {exc!r}")


def _build_cycle(board):
    logger = board["logger"]

    def return_ready_fn():
        try:
            result = move_to_named(
                board["robot"], board["arm"], logger, board["params"], "ready",
            )
            return Status.SUCCESS if result.ok else Status.FAILURE
        except Exception as exc:
            logger.error(f"sort_tree: return-ready raised: {exc!r}")
            return Status.FAILURE

    return_ready = Leaf("ReturnReady", return_ready_fn, logger)
    board["return_ready"] = return_ready

    def get_fresh_detections():
        board["staging_clear"] = False
        board["ready_after_failure"] = False
        board.pop("obj", None)
        board.pop("rec", None)
        board.pop("route", None)

        objects = board["cache"].get_fresh()
        pending = []
        for obj in objects:
            if _in_a_tray(obj) is not None:
                continue
            record = board["tracker"].get(obj)
            if record["attempts"] < MAX_ATTEMPTS_PER_OBJECT:
                pending.append((record, obj))
        logger.info(
            f"sort_tree: loop {board['loop']}: {len(objects)} seen, "
            f"{len(pending)} pending"
        )
        if not pending:
            board["staging_clear"] = True
            logger.info("sort_tree: staging area clear")
            return Status.FAILURE

        record, obj = pending[0]
        route = TRAY_FOR_COLOUR.get(obj["colour"])
        if route is None:
            attempt = board["tracker"].mark_attempt(record, obj)
            logger.warn(
                f"sort_tree: no tray for colour '{obj['colour']}' -- leaving it"
            )
            board["outcomes"].append((obj["cls"], "no_tray", None))
            _record_trial(
                board, obj, None, attempt, False, "no_tray", None, None,
                0.0, "no_tray",
            )
            return Status.FAILURE

        board.update(obj=obj, rec=record, route=route)
        return Status.SUCCESS

    def pick_object():
        obj = board["obj"]
        record = board["rec"]
        tray_name = board["route"][0]
        if record["attempts"] >= MAX_ATTEMPTS_PER_OBJECT:
            logger.warn(
                f"sort_tree: retry budget exhausted for {obj['cls']} "
                f"({record['attempts']}/{MAX_ATTEMPTS_PER_OBJECT})"
            )
            return Status.FAILURE

        # A prior failed Retry tick may have homed successfully.  Once a new pick
        # starts, that fact is stale: a later Place failure must trigger recovery.
        board["ready_after_failure"] = False
        attempt = board["tracker"].mark_attempt(record, obj)
        board["attempt"] = attempt
        board["attempt_started"] = time.time()
        logger.info(
            f"sort_tree: {obj['cls']} @ ({obj['x']:.3f},{obj['y']:.3f}) -> "
            f"{tray_name} (attempt {attempt}/{MAX_ATTEMPTS_PER_OBJECT})"
        )
        try:
            result = pick(
                board["robot"], board["arm"], logger, board["params"],
                board["twin"], (obj["x"], obj["y"], obj["z"]),
                expect_half_width=EXPECT_HALF_WIDTH, width_tol=WIDTH_TOL,
            )
            board["pick_result"] = result
            if result.ok:
                return Status.SUCCESS
            stage = result.stage
        except Exception as exc:
            logger.error(f"sort_tree: pick raised for {obj['cls']}: {exc!r}")
            stage = "exception"

        outcome = f"pick_{stage}"
        logger.error(f"sort_tree: pick failed ({stage}) for {obj['cls']}")
        board["outcomes"].append((obj["cls"], outcome, tray_name))
        _record_trial(
            board, obj, tray_name, attempt, False, stage, None, None,
            time.time() - board["attempt_started"], outcome,
        )
        board["ready_after_failure"] = (
            return_ready.tick() is Status.SUCCESS
        )
        return Status.FAILURE

    def place_object():
        obj = board["obj"]
        tray_name, x, y, z = board["route"]
        pick_result = board["pick_result"]
        try:
            result = place(
                board["robot"], board["arm"], logger, board["params"],
                board["twin"], (x, y, z),
                clearance=TRAY_RELEASE_CLEARANCE,
            )
            place_ok = result.ok
            place_stage = result.stage
        except Exception as exc:
            logger.error(f"sort_tree: place raised for {obj['cls']}: {exc!r}")
            place_ok = False
            place_stage = "exception"

        outcome = "placed" if place_ok else f"place_{place_stage}"
        board["outcomes"].append((obj["cls"], outcome, tray_name))
        _record_trial(
            board, obj, tray_name, board["attempt"], True, pick_result.stage,
            place_ok, place_stage, time.time() - board["attempt_started"], outcome,
        )
        if not place_ok:
            logger.error(
                f"sort_tree: place failed ({place_stage}) for {obj['cls']}"
            )
            return Status.FAILURE
        return Status.SUCCESS

    get_leaf = Leaf("GetFreshDetections", get_fresh_detections, logger)
    pick_leaf = Leaf("Pick", pick_object, logger)
    place_leaf = Leaf("Place", place_object, logger)
    cycle = Sequence("SortOneObject", [
        get_leaf,
        Retry(pick_leaf, MAX_ATTEMPTS_PER_OBJECT,
              name="RetryPick(2)", logger=logger),
        place_leaf,
        return_ready,
    ], logger)
    return cycle, return_ready


def main():
    argv = sys.argv[1:]
    max_objects = None
    if "--max" in argv:
        try:
            max_objects = int(argv[argv.index("--max") + 1])
        except (IndexError, ValueError):
            pass

    robot, arm, logger, params, twin = setup(node_name="sort_tree_sorting_line")
    try:
        add_sorting_line_collision(robot, logger)
    except Exception as exc:
        logger.warn(f"sort_tree: collision scene skipped: {exc!r}")

    cache = _DetectionCache(twin, logger)
    tracker = _ObjectTracker()
    conn, run_id = _open_db()
    try:
        from twin_moveit_scripts import trial_db
    except Exception:
        trial_db = None

    board = {
        "robot": robot,
        "arm": arm,
        "logger": logger,
        "params": params,
        "twin": twin,
        "cache": cache,
        "tracker": tracker,
        "outcomes": [],
        "conn": conn,
        "run_id": run_id,
        "trial_db": trial_db,
        "seq": 0,
        "loop": 0,
        "staging_clear": False,
        "ready_after_failure": False,
    }
    cycle, return_ready = _build_cycle(board)
    run_started = time.time()

    try:
        return_ready.tick()
        for loop in range(LOOP_GUARD):
            board["loop"] = loop
            if max_objects is not None and len(board["outcomes"]) >= max_objects:
                logger.info(f"sort_tree: reached --max {max_objects}")
                break

            # Leaves are synchronous. RUNNING here means Retry has scheduled its
            # next bounded Pick attempt, so re-tick the same memory Sequence.
            for _ in range(MAX_ATTEMPTS_PER_OBJECT + 2):
                status = cycle.tick()
                if status is not Status.RUNNING:
                    break
            else:
                logger.error("sort_tree: cycle tick guard tripped")
                status = Status.FAILURE
                cycle.reset()

            if status is Status.FAILURE:
                if board["staging_clear"]:
                    break
                if not board["ready_after_failure"]:
                    return_ready.tick()
        else:
            logger.error(f"sort_tree: LOOP_GUARD={LOOP_GUARD} tripped")
    finally:
        try:
            return_ready.tick()
        except Exception as exc:
            logger.warn(f"sort_tree: final home raised: {exc!r}")

        outcomes = board["outcomes"]
        placed = sum(1 for _, status, _ in outcomes if status == "placed")
        if conn and trial_db:
            try:
                trial_db.finish_run(conn, run_id, placed, time.time() - run_started)
                conn.close()
            except Exception as exc:
                print(f"sort_tree_sorting_line: DB finish failed ({exc!r})")

        print("\n===== SORT LINE TREE OUTCOMES =====")
        for cls, status, tray_name in outcomes:
            print(f"SORTTREE {cls:14s} -> {str(tray_name):12s} {status}")
        print(
            f"===== {placed}/{len(outcomes)} placed "
            "(ground truth decides) ====="
        )
        sys.stdout.flush()
        shutdown(twin)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""A5 (FR-9): durable SQLite record of every sort trial and its ground-truth outcome.

Why a DB and not just logs: this project's rule #2 is "judge by ground truth, never by
logs" -- every status line has lied at least once. So the benchmark keeps TWO independent
columns per run: what the task *believed* happened (logged_placed, per-trial pick/place
stages) and what *actually* happened in Gazebo (truth_correct / the `truth` table, written
by the harness from /world/empty/pose/info). metrics then reports both and flags the
divergence -- the divergence IS a headline result, not noise.

Three tables:
  runs    -- one row per sort invocation (scenario, timing, both tallies)
  trials  -- one row per pick-and-place ATTEMPT the task made (its own belief + timing)
  truth   -- one row per object per phase (before/after), from Gazebo poses (the arbiter)

Everything here is best-effort and non-fatal: a logging failure must never break a sort.
The writer is the sort task; the truth rows and run summary are written by the harness.
"""
import os
import sqlite3
import time

DEFAULT_DB = os.path.expanduser("~/ros_test_ws_mirror/logs/v2/benchmark.db")

_SCHEMA = """
CREATE TABLE IF NOT EXISTS runs (
    run_id        TEXT PRIMARY KEY,
    scenario      TEXT,
    ts_start      TEXT,
    ts_end        TEXT,
    launch        TEXT,
    n_present     INTEGER,   -- objects present at run start  (ground truth, harness)
    logged_placed INTEGER,   -- the task's OWN 'placed' tally  (belief)
    truth_correct INTEGER,   -- objects in the correct bin     (ground truth, harness)
    truth_total   INTEGER,   -- objects seen at the end        (ground truth, harness)
    wall_seconds  REAL,
    notes         TEXT
);
CREATE TABLE IF NOT EXISTS trials (
    id            INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id        TEXT,
    seq           INTEGER,   -- order the task attempted objects within the run
    cls           TEXT,      -- detector class_id, e.g. 'red_cube'
    colour        TEXT,
    shape         TEXT,
    det_x REAL, det_y REAL, det_z REAL,
    target_bin    TEXT,
    attempt       INTEGER,   -- 1-based attempt count for this physical object
    pick_ok       INTEGER,
    pick_stage    TEXT,
    place_ok      INTEGER,
    place_stage   TEXT,
    cycle_seconds REAL,
    outcome       TEXT       -- 'placed' | 'pick_<stage>' | 'place_<stage>' | 'no_bin'
);
CREATE TABLE IF NOT EXISTS truth (
    id            INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id        TEXT,
    phase         TEXT,      -- 'before' | 'after'
    obj_name      TEXT,      -- Gazebo entity, obj_<colour>_<shape>_<n>
    colour        TEXT,
    x REAL, y REAL, z REAL,
    got_bin       TEXT,
    want_bin      TEXT,
    correct       INTEGER
);
"""


def connect(path=None):
    """Open (creating if needed) the benchmark DB with the schema applied. WAL so a
    reader (metrics) never blocks the writer (sort task)."""
    path = path or os.environ.get("TWIN_DB") or DEFAULT_DB
    os.makedirs(os.path.dirname(path), exist_ok=True)
    conn = sqlite3.connect(path, timeout=10.0)
    conn.execute("PRAGMA journal_mode=WAL")
    conn.executescript(_SCHEMA)
    conn.commit()
    return conn


def start_run(conn, run_id, scenario, launch):
    conn.execute(
        "INSERT OR REPLACE INTO runs(run_id, scenario, ts_start, launch) "
        "VALUES(?,?,?,?)",
        (run_id, scenario, _now(), launch))
    conn.commit()


def record_trial(conn, run_id, seq, det, target_bin, attempt,
                 pick_ok, pick_stage, place_ok, place_stage,
                 cycle_seconds, outcome):
    conn.execute(
        "INSERT INTO trials(run_id, seq, cls, colour, shape, det_x, det_y, det_z, "
        "target_bin, attempt, pick_ok, pick_stage, place_ok, place_stage, "
        "cycle_seconds, outcome) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
        (run_id, seq, det.get("cls"), det.get("colour"), det.get("shape"),
         det.get("x"), det.get("y"), det.get("z"), target_bin, attempt,
         int(bool(pick_ok)), pick_stage,
         None if place_ok is None else int(bool(place_ok)), place_stage,
         cycle_seconds, outcome))
    conn.commit()


def finish_run(conn, run_id, logged_placed, wall_seconds, notes=""):
    conn.execute(
        "UPDATE runs SET ts_end=?, logged_placed=?, wall_seconds=?, notes=? "
        "WHERE run_id=?",
        (_now(), logged_placed, wall_seconds, notes, run_id))
    conn.commit()


def record_truth(conn, run_id, phase, rows):
    """rows: iterable of (obj_name, colour, x, y, z, got_bin, want_bin, correct)."""
    conn.executemany(
        "INSERT INTO truth(run_id, phase, obj_name, colour, x, y, z, got_bin, "
        "want_bin, correct) VALUES(?,?,?,?,?,?,?,?,?,?)",
        [(run_id, phase) + tuple(r) for r in rows])
    conn.commit()


def set_truth_summary(conn, run_id, n_present, truth_correct, truth_total):
    conn.execute(
        "UPDATE runs SET n_present=?, truth_correct=?, truth_total=? WHERE run_id=?",
        (n_present, truth_correct, truth_total, run_id))
    conn.commit()


def _now():
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())

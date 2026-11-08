#!/usr/bin/env python3
"""A5 benchmark: the ground-truth recorder and the metrics report.

Two jobs, one file so there is a single place the evidence layer lives:

  benchmark.py truth --run <id> --phase before|after [--db PATH]
      Read Gazebo object poses (the independent arbiter -- rule #2), work out which bin
      each object is in and which it SHOULD be in, and write one `truth` row per object.
      On 'after' it also fills the run summary (n_present / truth_correct / truth_total).

  benchmark.py report [--db PATH] [--scenario S1|S2]
      Aggregate the DB into the metrics the roadmap asks for: per-class accuracy, pick
      rate, ground-truth place-in-correct-bin rate, the logged-vs-truth divergence (how
      often a status line lied), cycle time and throughput.

Ground truth comes from tools/check_sort.py (imported, not duplicated): gz_objects(),
which_bin(), BIN_FOR_COLOUR. The DB layer is twin_moveit_scripts.trial_db.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_sort import gz_objects, which_bin, BIN_FOR_COLOUR  # noqa: E402
from twin_moveit_scripts import trial_db  # noqa: E402

RESTING_Z = 0.015
Z_TOL = 0.02


def _truth_rows():
    """One tuple per object: (name, colour, x, y, z, got_bin, want_bin, correct)."""
    rows = []
    for name, (x, y, z) in sorted(gz_objects().items()):
        colour = name.split("_")[1]
        want = BIN_FOR_COLOUR.get(colour, (None,))[0]
        got = which_bin(x, y)
        correct = int(got == want and abs(z - RESTING_Z) <= Z_TOL)
        rows.append((name, colour, x, y, z, got, want, correct))
    return rows


def cmd_truth(args):
    conn = trial_db.connect(args.db)
    rows = _truth_rows()
    trial_db.record_truth(conn, args.run, args.phase, rows)
    if args.phase == "after":
        n_before = conn.execute(
            "SELECT COUNT(*) FROM truth WHERE run_id=? AND phase='before'",
            (args.run,)).fetchone()[0]
        total = len(rows)
        correct = sum(r[7] for r in rows)
        trial_db.set_truth_summary(conn, args.run, n_before or total, correct, total)
        print(f"[truth after] {args.run}: {correct}/{total} in correct bin")
    else:
        print(f"[truth before] {args.run}: {len(rows)} objects present")
    conn.close()
    return 0


def _pct(n, d):
    return f"{100.0 * n / d:5.1f}%" if d else "   n/a"


def cmd_report(args):
    conn = trial_db.connect(args.db)
    conn.row_factory = __import__("sqlite3").Row
    scen_filter = (" WHERE scenario=?", (args.scenario,)) if args.scenario else ("", ())
    scenarios = [r["scenario"] for r in conn.execute(
        f"SELECT DISTINCT scenario FROM runs{scen_filter[0]} ORDER BY scenario",
        scen_filter[1])]
    if not scenarios:
        print("no runs in DB:", args.db or trial_db.DEFAULT_DB)
        return 1

    print(f"\n================ A5 SORTING BENCHMARK ({args.db or trial_db.DEFAULT_DB})")
    for scen in scenarios:
        runs = conn.execute(
            "SELECT * FROM runs WHERE scenario=? AND ts_end IS NOT NULL", (scen,)
        ).fetchall()
        if not runs:
            continue
        rids = [r["run_id"] for r in runs]
        qmarks = ",".join("?" * len(rids))

        tot_total = sum(r["truth_total"] or 0 for r in runs)
        tot_correct = sum(r["truth_correct"] or 0 for r in runs)
        tot_logged = sum(r["logged_placed"] or 0 for r in runs)
        tot_wall = sum(r["wall_seconds"] or 0.0 for r in runs)
        # Objects the task actually tried to sort (distinct physical object per run with a
        # bin target). For S1 this is 1/run, so it separates "sorted the targeted object"
        # from the deliberately-unsorted remainder that inflates the all-present denom.
        attempted = conn.execute(
            f"SELECT COUNT(DISTINCT run_id || '|' || colour) FROM trials "
            f"WHERE run_id IN ({qmarks}) AND target_bin IS NOT NULL", rids).fetchone()[0]

        trials = conn.execute(
            f"SELECT * FROM trials WHERE run_id IN ({qmarks})", rids).fetchall()
        n_trials = len(trials)
        picks_ok = sum(t["pick_ok"] or 0 for t in trials)
        a1 = [t for t in trials if t["attempt"] == 1]
        a1_pick_ok = sum(t["pick_ok"] or 0 for t in a1)
        cyc = [t["cycle_seconds"] for t in trials if t["pick_ok"] and t["cycle_seconds"]]
        mean_cyc = sum(cyc) / len(cyc) if cyc else 0.0

        print(f"\n--- {scen}  ({len(runs)} run(s), {tot_total} objects presented, "
              f"{attempted} attempted) ---")
        print(f"  correct-bin of ATTEMPTED (truth)    : {tot_correct}/{attempted}  "
              f"{_pct(tot_correct, attempted)}")
        print(f"  correct-bin of all present (truth)  : {tot_correct}/{tot_total}  "
              f"{_pct(tot_correct, tot_total)}")
        print(f"  task-reported placed (belief)       : {tot_logged}/{tot_total}  "
              f"{_pct(tot_logged, tot_total)}")
        div = tot_logged - tot_correct
        print(f"  logged-vs-truth divergence          : {div:+d} object(s)"
              + ("   <- a status line lied" if div else "   (honest)"))
        print(f"  pick success (all attempts)         : {picks_ok}/{n_trials}  "
              f"{_pct(picks_ok, n_trials)}")
        print(f"  pick success (FIRST attempt only)   : {a1_pick_ok}/{len(a1)}  "
              f"{_pct(a1_pick_ok, len(a1))}")
        print(f"  mean pick-place cycle               : {mean_cyc:5.1f} s")
        if tot_wall:
            print(f"  throughput                          : "
                  f"{60.0 * tot_correct / tot_wall:5.2f} objects/min "
                  f"({tot_wall / max(1, len(runs)):.0f} s/run)")

        # per-shape first-attempt pick rate -- this is where cylinders show up
        print("  first-attempt pick by shape:")
        for shape in sorted({t["shape"] for t in a1 if t["shape"]}):
            s = [t for t in a1 if t["shape"] == shape]
            ok = sum(t["pick_ok"] or 0 for t in s)
            print(f"     {shape:9s} {ok}/{len(s)}  {_pct(ok, len(s))}")

        # per-colour ground-truth correctness (final poses)
        print("  correct-bin by colour (ground truth):")
        rows = conn.execute(
            f"SELECT colour, SUM(correct) c, COUNT(*) n FROM truth "
            f"WHERE phase='after' AND run_id IN ({qmarks}) GROUP BY colour ORDER BY colour",
            rids).fetchall()
        for r in rows:
            print(f"     {r['colour']:9s} {r['c']}/{r['n']}  {_pct(r['c'], r['n'])}")
    print()
    conn.close()
    return 0


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    t = sub.add_parser("truth")
    t.add_argument("--run", required=True)
    t.add_argument("--phase", required=True, choices=["before", "after"])
    t.add_argument("--db", default=None)
    r = sub.add_parser("report")
    r.add_argument("--db", default=None)
    r.add_argument("--scenario", default=None)
    args = ap.parse_args()
    if args.cmd == "truth":
        return cmd_truth(args)
    return cmd_report(args)


if __name__ == "__main__":
    sys.exit(main())

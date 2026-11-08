#!/usr/bin/env python3
"""A2a reach probe: can the arm actually reach each candidate bin / tray pose?

The 3 bins at the old PLACE_SLOTS leave only ~0.08 x 0.18 m of free workspace, which
is not enough to stage 5-6 objects. The proposed re-layout moves the bins to a BACK
ROW and puts the staging tray in a FRONT STRIP -- but those positions sit outside what
has ever been reach-verified. This probes them before any sorting logic is built on top.

It drives the REAL grasp posture (seeded-IK top-down, same as pick()), at both the
approach height and the grasp height, so a pass here means pick/place can genuinely
work there -- not merely that some IK solution exists.

Run with the sim already up (see tools/run_reach_probe.sh).
Output lines: "REACH <name> (x, y) approach=OK|FAIL grasp=OK|FAIL"
"""
import sys

from twin_moveit_scripts.move_to import (
    setup, shutdown, move_to_named, move_to_grasp_pose, move_gripper,
    TOPDOWN_GRASP_QUAT, FINGER_REACH_Z, APPROACH_CLEARANCE, GRIPPER_MAX,
)

RESTING_Z = 0.015  # the geometry invariant: object centre on the ground plane

# Run every candidate more than once. A genuinely unreachable pose fails EVERY pass
# (IK/planning fails deterministically); an intermittent execution miss passes at
# least once. Without this you cannot tell a reachability limit from flakiness --
# the first probe reported (0.50,0) unreachable, which is the verified spawn point.
PASSES = 2

# ROUND 2 -- lock the final A2 layout.
# Round 1 (2 passes) proved: bins at (0.37, +/-0.20) are GRASP-Z BLOCKED 0/2 (the arm
# stalls 28-30 mm high, deterministically, though IK solves), while (0.45, +/-0.18),
# (0.37, 0), (0.52, +/-0.15) and (0.58, 0) are all 2/2. The reachable region narrows as
# |y| grows. So KEEP the three existing bins (already verified) and put the staging row
# out FRONT at x ~ 0.56, clear of the bins' x-extent (<=0.52) and |y| <= 0.10 so it
# never overlaps the side bins (y >= 0.11). This probes that exact row -- five object
# slots -- instead of interpolating between verified points.
CANDIDATES = [
    ("CTRL_bin_a",    0.45,  0.18),   # existing bin, control
    ("CTRL_bin_c",    0.37,  0.00),   # existing bin, control
    ("TRAY_ny10",     0.56, -0.10),   # staging row, 5 object slots
    ("TRAY_ny05",     0.56, -0.05),
    ("TRAY_mid",      0.56,  0.00),
    ("TRAY_py05",     0.56,  0.05),
    ("TRAY_py10",     0.56,  0.10),
]


def main():
    robot, arm, logger, params, twin = setup(node_name="reach_probe")
    tally = {name: {"appr": 0, "grasp": 0, "ikfail": 0} for name, _, _ in CANDIDATES}
    try:
        z_grasp = RESTING_Z + FINGER_REACH_Z
        z_appr = z_grasp + APPROACH_CLEARANCE
        logger.info(f"reach_probe: z_grasp={z_grasp:.3f} z_approach={z_appr:.3f} passes={PASSES}")

        # Faithful to pick(): the hand is OPEN for every descend. The first probe
        # skipped this and descended with whatever finger state startup left behind.
        move_to_named(robot, arm, logger, params, "ready")
        g0 = move_gripper(robot, arm, logger, params, twin, GRIPPER_MAX)
        logger.info(f"reach_probe: pre-open gripper -> {g0}")

        for p in range(1, PASSES + 1):
            for name, x, y in CANDIDATES:
                # always start from the same well-conditioned basin, exactly like pick()
                home = move_to_named(robot, arm, logger, params, "ready")
                if not home.ok:
                    logger.error(f"reach_probe: could not home before {name}: {home}")
                    continue

                a = move_to_grasp_pose(robot, arm, logger, params, x, y, z_appr, TOPDOWN_GRASP_QUAT)
                if a.ok:
                    tally[name]["appr"] += 1
                    g = move_to_grasp_pose(robot, arm, logger, params, x, y,
                                           z_grasp, TOPDOWN_GRASP_QUAT)
                    if g.ok:
                        tally[name]["grasp"] += 1
                    else:
                        logger.warn(f"REACH pass{p} {name} grasp miss: {g}")
                logger.info(f"REACH pass{p} {name} ({x}, {y}) "
                            f"appr={tally[name]['appr']} grasp={tally[name]['grasp']}")

        move_to_named(robot, arm, logger, params, "ready")
    finally:
        print(f"\n===== REACH PROBE SUMMARY (passes={PASSES}) =====")
        solid = 0
        for name, x, y in CANDIDATES:
            t = tally[name]
            # reachable = planning/IK never refused; flaky = some passes missed arrival
            verdict = ("REACHABLE" if t["grasp"] == PASSES else
                       "FLAKY" if t["grasp"] > 0 else
                       "UNREACHABLE" if t["appr"] == 0 else "GRASP-Z BLOCKED")
            solid += 1 if t["grasp"] == PASSES else 0
            print(f"REACH {name:15s} ({x:+.2f}, {y:+.2f})  "
                  f"approach={t['appr']}/{PASSES}  grasp={t['grasp']}/{PASSES}  -> {verdict}")
        print(f"===== {solid}/{len(CANDIDATES)} solid on every pass =====")
        sys.stdout.flush()
        shutdown(twin)


if __name__ == "__main__":
    main()

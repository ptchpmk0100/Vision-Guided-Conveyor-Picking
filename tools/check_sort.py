#!/usr/bin/env python3
"""A4 ground truth: did every object actually END UP in its correct bin?

The sort task prints its own "N/M placed" tally, but in this project a status line has
lied at least three times (HELD on an empty hand, place=True on an unmoved cube,
ok [reached] on a gripper that never moved). So the verdict comes from Gazebo poses.

Reads /world/empty/pose/info, finds every obj_<colour>_<shape>_<n>, works out which bin
footprint it is sitting in, and compares that with the bin its COLOUR should route to.

Usage (sim running):  python3 tools/check_sort.py
Exit 0 only if every object is in its correct bin.
"""
import math
import re
import subprocess
import sys

BIN_FOR_COLOUR = {
    "red":    ("bin_a", 0.45, 0.18),
    "green":  ("bin_b", 0.45, -0.18),
    "yellow": ("bin_c", 0.37, 0.00),
}
BIN_RADIUS = 0.075       # inside this of a bin centre == in that bin
RESTING_Z = 0.015
Z_TOL = 0.02             # must still be resting on the ground, not stuck on a wall


def gz_objects():
    try:
        out = subprocess.run(
            ["gz", "topic", "-e", "-n", "1", "-t", "/world/empty/pose/info"],
            capture_output=True, text=True, timeout=15).stdout
    except Exception as e:
        print(f"!! could not read gz ground truth: {e}")
        return {}
    objs, name, in_pos, pos = {}, None, False, {}
    for line in out.splitlines():
        s = line.strip()
        m = re.match(r'name:\s*"([^"]+)"', s)
        if m:
            name, pos, in_pos = m.group(1), {}, False
            continue
        if s.startswith("position"):
            in_pos = True
            continue
        if in_pos:
            m = re.match(r'([xyz]):\s*(-?[0-9.eE+-]+)', s)
            if m:
                pos[m.group(1)] = float(m.group(2))
                if len(pos) == 3:
                    if name and name.startswith("obj_"):
                        objs[name] = (pos["x"], pos["y"], pos["z"])
                    in_pos = False
    return objs


def which_bin(x, y):
    for colour, (bname, bx, by) in BIN_FOR_COLOUR.items():
        if math.hypot(x - bx, y - by) <= BIN_RADIUS:
            return bname
    return None


def main():
    objs = gz_objects()
    print("\n===== SORT GROUND TRUTH =====")
    if not objs:
        print("!! no obj_* entities found")
        return 1
    right = 0
    for name in sorted(objs):
        x, y, z = objs[name]
        colour = name.split("_")[1]
        want = BIN_FOR_COLOUR.get(colour, (None,))[0]
        got = which_bin(x, y)
        z_ok = abs(z - RESTING_Z) <= Z_TOL
        if got == want and z_ok:
            right += 1
            verdict = "OK"
        elif got is None:
            verdict = "NOT-SORTED"
        elif got != want:
            verdict = f"WRONG-BIN(in {got})"
        else:
            verdict = f"BAD-Z({z:.3f})"
        print(f"{verdict:18s} {name:22s} pos=({x:+.3f},{y:+.3f},{z:+.3f}) want={want}")
    print(f"===== {right}/{len(objs)} objects in the correct bin =====")
    return 0 if right == len(objs) else 1


if __name__ == "__main__":
    sys.exit(main())

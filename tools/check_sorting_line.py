#!/usr/bin/env python3
"""Sorting-line ground truth (Stage A geometry gate, reused later).

Reads /world/empty/pose/info and verifies the raised scene by Gazebo poses (never logs):
  * the 3 colour boxes settled with centre z=0.320 at their staging XY;
  * no legacy ground objects (obj_*_cube) or ground bins (bin_a/b/c) leaked into the world;
  * the trays (tray_red/green/yellow) and pick_belt are present.

Usage (sim running):  python3 tools/check_sorting_line.py
Exit 0 iff the 3 boxes are within tolerance and nothing legacy is present.
"""
import re
import subprocess
import sys

STAGING = {   # colour -> (x, y) belt-height staging pose
    "red":    (0.560,  0.110),
    "green":  (0.560,  0.000),
    "yellow": (0.560, -0.110),
}
TRAY_FOR_COLOUR = {   # colour -> (tray name, x, y) at belt height
    "red":    ("tray_red", 0.450, 0.180),
    "green":  ("tray_green", 0.450, -0.180),
    "yellow": ("tray_yellow", 0.370, 0.000),
}
TRAY_RADIUS = 0.075
BOX_CENTER_Z = 0.320
Z_TOL = 0.006          # settled centre height tolerance
XY_TOL = 0.020         # settling drift tolerance in x/y
LEGACY = re.compile(r"^(bin_[abc]|obj_\w+_cube_\d+|obj_\w+_cylinder_\d+|main_belt|feeder_belt|collection_tray|marker_\w+|floor_stripe)$")


def _parse(topic):
    """Proven inline parser (same shape as tools/check_sort.py): record a pose the moment
    its 3 position components are seen, then stop reading until the next name -- so the
    following orientation/id lines are ignored and the LAST value for a name wins."""
    try:
        out = subprocess.run(["gz", "topic", "-e", "-n", "1", "-t", topic],
                             capture_output=True, text=True, timeout=15).stdout
    except Exception:
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
            mm = re.match(r'([xyz]):\s*(-?[0-9.eE+-]+)', s)
            if mm:
                pos[mm.group(1)] = float(mm.group(2))
                if len(pos) == 3:
                    if name:
                        objs[name] = (pos["x"], pos["y"], pos["z"])
                    in_pos = False
    return objs


def read_poses():
    # dynamic_pose/info carries the authoritative CURRENT pose of moved objects (the boxes);
    # pose/info carries the static models (trays, deck). Merge, dynamic wins.
    poses = _parse("/world/empty/pose/info")
    poses.update(_parse("/world/empty/dynamic_pose/info"))
    return poses


def _which_tray(x, y):
    for colour, (tname, tx, ty) in TRAY_FOR_COLOUR.items():
        if ((x - tx) ** 2 + (y - ty) ** 2) ** 0.5 <= TRAY_RADIUS:
            return tname
    return None


def check_placed(poses):
    """Post-sort verdict: every box in its COLOUR's tray, resting at belt height."""
    boxes = {n: p for n, p in poses.items() if re.match(r"^obj_\w+_box_\d+$", n)}
    print("\n===== SORTING-LINE GROUND TRUTH (placed) =====")
    if not boxes:
        print("!! no obj_*_box_* entities found")
        return 1
    right = 0
    for name in sorted(boxes):
        x, y, z = boxes[name]
        colour = name.split("_")[1]
        want = TRAY_FOR_COLOUR.get(colour, (None,))[0]
        got = _which_tray(x, y)
        z_ok = abs(z - BOX_CENTER_Z) <= 0.020
        if got == want and z_ok:
            right += 1
            v = "OK"
        elif got is None:
            v = "NOT-SORTED"
        elif got != want:
            v = f"WRONG-TRAY(in {got})"
        else:
            v = f"BAD-Z({z:.3f})"
        print(f"{v:18s} {name:18s} pos=({x:+.3f},{y:+.3f},{z:+.3f}) want={want}")
    print(f"===== {right}/{len(boxes)} boxes in the correct tray =====")
    return 0 if right == len(boxes) == 3 else 1


def main():
    if "--placed" in sys.argv:
        return check_placed(read_poses())
    poses = read_poses()
    print("\n===== SORTING-LINE GROUND TRUTH (Stage A) =====")
    boxes = {n: p for n, p in poses.items() if re.match(r"^obj_\w+_box_\d+$", n)}
    if not boxes:
        print("!! no obj_*_box_* entities found")
        return 1

    ok = 0
    for name in sorted(boxes):
        x, y, z = boxes[name]
        colour = name.split("_")[1]
        want = STAGING.get(colour)
        if want is None:
            print(f"UNKNOWN-COLOUR {name} pos=({x:+.3f},{y:+.3f},{z:+.3f})")
            continue
        dxy = ((x - want[0]) ** 2 + (y - want[1]) ** 2) ** 0.5
        z_ok = abs(z - BOX_CENTER_Z) <= Z_TOL
        xy_ok = dxy <= XY_TOL
        verdict = "OK" if (z_ok and xy_ok) else (
            f"BAD-Z({z:.3f})" if not z_ok else f"DRIFT({dxy*1000:.0f}mm)")
        if z_ok and xy_ok:
            ok += 1
        print(f"{verdict:14s} {name:18s} pos=({x:+.3f},{y:+.3f},{z:+.3f}) "
              f"want=({want[0]:+.3f},{want[1]:+.3f},{BOX_CENTER_Z:.3f})")

    legacy = sorted(n for n in poses if LEGACY.match(n))
    trays = sorted(n for n in poses if n.startswith("tray_"))
    deck = [n for n in poses if n == "pick_belt"]
    print(f"--- trays present: {trays or 'NONE (static may be omitted from pose/info)'}")
    print(f"--- pick_belt present: {bool(deck)}")
    if legacy:
        print(f"!! LEGACY entities leaked into the new world: {legacy}")
    print(f"===== {ok}/{len(boxes)} boxes settled correctly; "
          f"legacy={'NONE' if not legacy else legacy} =====")
    return 0 if (ok == len(boxes) == 3 and not legacy) else 1


if __name__ == "__main__":
    sys.exit(main())

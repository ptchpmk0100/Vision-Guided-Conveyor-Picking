#!/usr/bin/env python3
"""Independent Gazebo verdict for condition routing: every parcel must end where its TRUE condition says.

damaged -> inside the reject bin; intact -> a distinct tote slot (+ the usual stable / upright / height checks).
A parcel still on the belt counts as 'left_on_belt' (a skipped parcel), never as correct. True conditions come from
the spawn-time variant manifest, positions from live Gazebo poses; the arm's own status messages are not used.

    python3 tools/check_condition_routing.py --manifest logs/condition/arm_x/manifest.json [--allow-skipped]
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/twin_moveit_scripts'))
from check_parcel_cell import poses  # noqa: E402
from twin_moveit_scripts.parcel_layout import PARCEL_Z, batch_layout  # noqa: E402
from twin_moveit_scripts.parcel_reject import inside_bin  # noqa: E402


def check(manifest, allow_skipped=False):
    truth = {p['name']: p['condition'] for p in manifest['parcels']}
    batch = batch_layout(manifest['count'])
    first = poses()
    time.sleep(1.0)
    second = poses()
    rows, used = [], set()
    for name in sorted(truth, key=lambda n: int(n.split('_')[1])):
        if name not in first or name not in second:
            rows.append({'name': name, 'true': truth[name], 'where': 'missing', 'passed': False})
            continue
        x, y, z, yaw, roll, pitch = second[name]
        stable = math.dist(first[name][:3], second[name][:3]) < .003
        upright = abs(roll) < .03 and abs(pitch) < .03
        height_ok = abs(z - PARCEL_Z) < .008
        if inside_bin(x, y, yaw):
            where = 'bin'
        elif batch.contains(x, y, yaw):
            where = 'tote'
        elif y > -3.7 and 0.43 < x:
            where = 'belt'
        else:
            where = 'elsewhere'
        row = {'name': name, 'true': truth[name], 'where': where, 'position': [round(x, 4), round(y, 4), round(z, 4)],
               'stable': stable, 'upright': upright, 'height_ok': height_ok}
        expected = 'bin' if truth[name] == 'damaged' else 'tote'
        ok = where == expected and stable and upright and height_ok
        if where == 'tote':
            err, slot = min((math.dist((x, y), xy), i) for i, xy in enumerate(batch.slots))
            row.update(slot=slot, slot_error_m=round(err, 4))
            ok = ok and err <= .015 and slot not in used
            used.add(slot)
        row['passed'] = ok
        rows.append(row)
    misrouted = [r['name'] for r in rows if r['where'] in ('bin', 'tote') and
                 r['where'] != ('bin' if r['true'] == 'damaged' else 'tote')]
    on_belt = [r['name'] for r in rows if r['where'] == 'belt']
    found = sorted(n for n in second if n.startswith('parcel_') and n[7:].isdigit())
    exact = found == sorted(truth)
    wrong = [r['name'] for r in rows if not r['passed'] and r['name'] not in on_belt]
    passed = exact and not misrouted and not wrong and (allow_skipped or not on_belt)
    return {'passed': passed, 'exact_entities': exact, 'correct': sum(r['passed'] for r in rows),
            'total': len(rows), 'misrouted': misrouted, 'left_on_belt': on_belt, 'failed_other': wrong,
            'damaged_in_bin': sum(r['where'] == 'bin' and r['true'] == 'damaged' for r in rows),
            'intact_in_tote': sum(r['where'] == 'tote' and r['true'] == 'intact' for r in rows),
            'parcels': rows}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--manifest', required=True)
    ap.add_argument('--allow-skipped', action='store_true')
    args = ap.parse_args()
    result = check(json.loads(Path(args.manifest).read_text()), args.allow_skipped)
    print(json.dumps(result, indent=1))
    sys.exit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()

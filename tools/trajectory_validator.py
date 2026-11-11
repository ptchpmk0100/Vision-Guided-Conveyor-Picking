#!/usr/bin/env python3
"""Replay sampled checks from a motion JSONL file. Does not authorize execution."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/twin_moveit_scripts'))
from twin_moveit_scripts.motion_audit import validate


def replay(path):
    results = []
    for row in map(json.loads, path.read_text().splitlines()):
        if row.get('event') != 'candidate':
            continue
        verdict = validate(row['trajectory'], row['limits'],
                           stationary_start=row.get('measured_start'))
        results.append(dict(command_id=row['command_id'], **verdict))
    return dict(sampled_checks_passed=bool(results) and all(
        r['sampled_checks_passed'] for r in results), candidates=results,
        execution_authorized=False, note='Collision requires a live scene; not replayed here.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('records', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = replay(args.records)
    text = json.dumps(result, indent=2) + '\n'
    if args.output:
        args.output.write_text(text)
    print(text, end='')
    raise SystemExit(0 if result['sampled_checks_passed'] else 1)

#!/usr/bin/env python3
"""Offline shadow-arrival replay with explicit profile and a single clock domain.

JSONL events: {event:command_started, command_id, target, now}, followed by
{event:state, now, stamp, positions, velocities, result_command_id,
 controller_succeeded, fault?}. Input must already map stamps to one clock domain.
"""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/twin_moveit_scripts'))
from twin_moveit_scripts.arrival_gate import ArrivalGate, ArrivalProfile


def replay(profile, rows):
    gate = ArrivalGate(ArrivalProfile(**profile))
    verdicts = []
    for row in rows:
        row = dict(row)
        event = row.pop('event')
        if event == 'command_started':
            gate.start(**row)
        elif event == 'state' and not gate.terminal:
            verdicts.append(dict(command_id=gate.command_id, now=row['now'], status=gate.update(**row)))
        else:
            raise ValueError(f'Unexpected or post-terminal replay event: {event}')
    return dict(execution_authorized=False, shadow_only=True, verdicts=verdicts)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--samples', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = replay(json.loads(args.profile.read_text()),
                    map(json.loads, args.samples.read_text().splitlines()))
    args.output.write_text(json.dumps(result, indent=2) + '\n')

#!/usr/bin/env python3
"""Summarize planned motion records. Actual placement remains the independent verdict."""
import argparse
import json
from pathlib import Path


def summarize(path):
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    candidates = [r for r in rows if r['event'] == 'candidate']
    execution = [r for r in rows if r['event'] == 'execution_finished']
    sent = {r['command_id'] for r in rows if r['event'] == 'execution_started'}
    executed = [r for r in candidates if r['command_id'] in sent]
    plans = [r for r in rows if r['event'] == 'plan_finished']
    cycles = []
    task_path = path.with_name('task.json')
    if task_path.exists():
        task = json.loads(task_path.read_text())
        start = task.get('started_wall_monotonic')
        events = task.get('events', [])
        if start is not None:
            for ready in (e for e in events if e['state'] == 'READY'):
                end = next((e for e in events if e['state'] == 'SETTLED'
                            and e.get('slot') == ready.get('slot')
                            and e['wall_elapsed'] >= ready['wall_elapsed']), None)
                if end is None:
                    continue
                group = [r for r in executed if ready['wall_elapsed'] <=
                         r['monotonic_ns']/1e9-start <= end['wall_elapsed']]
                cycles.append(dict(slot=ready.get('slot'), dispatched=len(group),
                    joint_path_length_rad=sum(r['metrics']['joint_path_length_rad'] for r in group),
                    ee_path_length_m=sum(r['metrics']['ee_path_length_m'] or 0 for r in group)))
    return dict(plans=len(plans), planning_wall_s=sum(r['planning_wall_s'] for r in plans),
                failed_plans=sum(not r['success'] for r in plans),
                fallbacks=sum(r['event'] == 'fallback' for r in rows),
                candidates=len(candidates), dispatched=len(executed),
                execution_returns=len(execution),
                planned_duration_sim=sum(r['metrics']['duration_sim'] for r in executed),
                joint_path_length_rad=sum(r['metrics']['joint_path_length_rad'] for r in executed),
                ee_path_length_m=sum(r['metrics']['ee_path_length_m'] or 0 for r in executed),
                interior_stop_samples=sum(r['metrics']['interior_stop_samples'] for r in executed),
                planners=sorted({r['planner_id'] for r in plans}),
                per_cycle_paths=cycles,
                per_plan=[dict(planner_id=r['planner_id'], kind=r['kind'],
                               planning_wall_s=r['planning_wall_s'], success=r['success']) for r in plans],
                validation_failures=[r['validation'] for r in candidates
                                     if r['validation'] and not r['validation']['sampled_checks_passed']],
                note='Planned path metrics; stops are sampled interior zeros, not measured robot stops. '
                     'Use task.json PARCEL_STATE intervals and existing benchmark.py for cycle/throughput.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('records', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = summarize(args.records)
    text = json.dumps(result, indent=2) + '\n'
    if args.output:
        args.output.write_text(text)
    print(text, end='')

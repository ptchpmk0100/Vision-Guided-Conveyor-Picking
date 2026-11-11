#!/usr/bin/env python3
"""Independent continuous-feed verdict from Gazebo observations and task times."""
import argparse
import json
from pathlib import Path
import statistics


def verdict(task, evidence):
    expected={f'parcel_{i}' for i in range(1,task.get('parcel_count',3)+1)}
    events = task['events']
    start = next(row['sim_time'] for row in events if row['state'] == 'BELT_RUNNING')+.2
    end = next(row['sim_time'] for row in events if row['state'] == 'FULL')-.2
    speed = task['belt_speed']
    commands = [row for row in evidence['belt'] if start <= row[0] <= end]
    moving = (len(commands) > 20 and all(abs(row[1]-speed) < 1e-6 for row in commands)
              and max(b[0]-a[0] for a, b in zip(commands, commands[1:])) < .5
              and commands[0][0]-start < .5 and end-commands[-1][0] < .5)
    intercepts = [row for row in events if row['state'] == 'INTERCEPT']
    lift_requests=[row for row in events if row['state']=='LIFT_REQUESTED']
    transport = []
    for index, intercept in enumerate(intercepts, 1):
        name = (lift_requests[index-1]['object'] if intercept.get('grasp_mode')=='confirmed'
                and index<=len(lift_requests) else f'parcel_{index}')
        # Use uncontacted approach travel only: a pinched carton is no longer a
        # free conveyor probe. Gazebo observations never enter runtime control.
        rows = [row for row in evidence['poses'] if row[1] == name
                and start <= row[0] <= intercept['start_sim'] and abs(row[2]-.56) < .025
                and abs(row[4]-.320) < .012]
        rates = [(b[3]-a[3])/(b[0]-a[0]) for a, b in zip(rows, rows[1:]) if b[0]-a[0] > .05]
        measured = statistics.median(rates) if rates else 0
        displacement = rows[-1][3]-rows[0][3] if len(rows) > 1 else 0
        lift_time=(lift_requests[index-1]['lift_sim'] if intercept.get('grasp_mode')=='confirmed'
                   else intercept['lift_sim'])
        following = [row for row in evidence['poses'] if row[1] == name
                     and intercept['grasp_sim']+.1 <= row[0] <= lift_time-.1]
        following_rates = [(b[3]-a[3])/(b[0]-a[0]) for a, b in zip(following, following[1:])
                           if b[0]-a[0] > .05]
        following_speed = statistics.median(following_rates) if following_rates else 0
        prediction_error = max((abs(row[3]-(intercept['target_at_start'][1]+
                                    intercept['velocity']*(row[0]-intercept['start_sim'])))
                                for row in following), default=float('inf'))
        observed_span = rows[-1][0]-rows[0][0] if len(rows) > 1 else 0.
        if task.get('parcel_count', 3) == 3:
            moved = displacement > .12           # original rule, unchanged
        else:
            # Detection-gated launch grips a box soon after it is tracked, so judge the
            # free travel by time: at least 0.8 s observed, moving at the commanded speed.
            moved = observed_span >= .8 and displacement >= .8*speed*observed_span
        transport.append(dict(name=name, measured_speed=measured, displacement=displacement,
                              observed_span=observed_span,
                              during_grasp_speed=following_speed, following_error_m=prediction_error,
                              passed=abs(measured-speed) < .008 and moved
                              and abs(following_speed-speed) < .008 and prediction_error < .012))
    return dict(passed=moving and len(transport) == len(expected)
                and {p['name'] for p in transport}==expected and all(v['passed'] for v in transport),
                always_commanded=moving, command_samples=len(commands), speed=speed,
                start_sim=start, end_sim=end, parcels=transport)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--directory', type=Path, required=True)
    args = parser.parse_args()
    result = verdict(json.loads((args.directory/'task.json').read_text()),
                     json.loads((args.directory/'transport.json').read_text()))
    (args.directory/'continuous_verdict.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)

#!/usr/bin/env python3
"""Own isolated simulator lifetimes; fail on actual verdicts and health failures."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import uuid
from twin_moveit_scripts.parcel_layout import batch_layout

ROOT = Path(__file__).resolve().parents[1]


def audit_finished(args):
    """Final audit restricted to this task's recorded completed run markers."""
    markers = {'baseline_20261109', 'baseline_host_20261109'}
    for path in (ROOT / 'logs/parcel_redesign').glob('*/run*/result.json'):
        marker = json.loads(path.read_text()).get('session', '')
        if marker.startswith(('parcel_', 'colour_regression_')):
            markers.add(marker)
    def owned():
        found = {}
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit() or int(entry.name) == os.getpid():
                continue
            try:
                values = (entry / 'environ').read_bytes().split(b'\0')
                for value in values:
                    if value.startswith(b'TWIN_PARCEL_SESSION='):
                        marker = value.split(b'=', 1)[1].decode()
                        if marker in markers:
                            found.setdefault(marker, []).append(int(entry.name))
            except (PermissionError, FileNotFoundError, ProcessLookupError):
                pass
        return found
    before = owned()
    signals = {marker: cleanup(marker) for marker in before}
    after = owned()
    args.logdir.mkdir(parents=True, exist_ok=False)
    result = dict(checked_finished_markers=sorted(markers), before=before,
                  signals=signals, remaining=after, passed=not after)
    (args.logdir / 'audit.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


def run_colours(args):
    """Keep the original harness intact and independently enforce all its verdicts."""
    args.logdir.mkdir(parents=True, exist_ok=False)
    results = []
    for number in range(args.runs):
        directory = args.logdir / f'run{number+1}'
        directory.mkdir()
        marker = 'colour_regression_' + uuid.uuid4().hex
        env = os.environ.copy()
        env.update(ROS_DOMAIN_ID=str(args.domain + number), GZ_PARTITION=marker,
                   TWIN_PARCEL_SESSION=marker, ROS_LOG_DIR=str(directory / 'ros'),
                   GZ_LOG_PATH=str(directory / 'gazebo'),
                   LOGDIR=str(directory), SORTEXE='sort_tree_sorting_line',
                   TWIN_EXTRINSIC=str(ROOT / 'src/twin_action_demo/config/overhead_camera_extrinsic.yaml'))
        record = {'session': marker, 'domain': env['ROS_DOMAIN_ID'], 'passed': False}
        try:
            with (directory / 'transcript.log').open('w') as out:
                rc = subprocess.run(['bash', '--noprofile', '--norc', 'tools/run_sort_line_check.sh', '1'],
                    env=env, cwd=ROOT, stdout=out, stderr=subprocess.STDOUT, timeout=1100).returncode
            transcript = (directory / 'transcript.log').read_text()
            if rc or '===== 3/3 boxes in the correct tray =====' not in transcript:
                raise RuntimeError('Original sorter independent placement verdict failed')
            for label in ('system ERRORs(non-task)', 'planning failures', 'tracebacks', 'fatal'):
                if not re.search(re.escape(label) + r': 0\b', transcript):
                    raise RuntimeError(f'Original sorter health gate failed: {label}')
            exits = re.findall(r'sort rc=(\d+)', transcript)
            log = (directory / 'sline_sort_1.log').read_text()
            if exits not in (['0'], ['245']) or '3/3 placed (ground truth decides)' not in log:
                raise RuntimeError('Original sorter completion/exit evidence failed')
            if exits == ['245'] and 'Deleting MoveItCpp' not in log:
                raise RuntimeError('Unrecognized original sorter teardown failure')
            record.update(passed=True, sorter_exit=int(exits[0]), placed=3)
        except Exception as exc:
            record['error'] = str(exc)
        finally:
            record['cleanup'] = cleanup(marker)
            (directory / 'result.json').write_text(json.dumps(record, indent=2) + '\n')
        results.append(record)
        print(json.dumps({k: v for k, v in record.items() if k != 'cleanup'}), flush=True)
        if not record['passed']:
            break
    (args.logdir / 'summary.json').write_text(json.dumps(results, indent=2) + '\n')
    return 0 if len(results) == args.runs and all(r['passed'] for r in results) else 1


def cleanup(marker):
    """Recheck per-process session ownership immediately before each signal."""
    key = f'TWIN_PARCEL_SESSION={marker}'.encode()
    stopped = []
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit() or int(entry.name) == os.getpid():
                continue
            try:
                if key in (entry / 'environ').read_bytes().split(b'\0'):
                    os.kill(int(entry.name), sig)
                    stopped.append((int(entry.name), int(sig)))
            except (PermissionError, FileNotFoundError, ProcessLookupError):
                pass
        time.sleep(1)
    return stopped


def run(args):
    batch=batch_layout(args.parcel_count,args.parcel_pitch)
    args.logdir.mkdir(parents=True, exist_ok=False)
    outcomes = []
    for number in range(args.runs):
        directory = args.logdir / f'run{number+1}'
        directory.mkdir()
        manifest = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                    for folder in ('src', 'tools') for path in (ROOT / folder).rglob('*')
                    if path.is_file() and path.suffix in ('.py', '.sdf', '.urdf', '.yaml', '.srdf')
                    and '__pycache__' not in path.parts}
        (directory / 'source_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        marker = 'parcel_' + uuid.uuid4().hex
        env = os.environ.copy()
        env.update(ROS_DOMAIN_ID=str(args.domain + number), GZ_PARTITION=marker,
                   TWIN_PARCEL_SESSION=marker, ROS_LOG_DIR=str(directory / 'ros'),
                   GZ_LOG_PATH=str(directory / 'gazebo'),
                   PYTHONDONTWRITEBYTECODE='1')
        env['TWIN_PARCEL_COUNT']=str(batch.count)
        env['TWIN_PARCEL_PITCH']=str(batch.pitch)
        if args.motion_metrics:
            env['TWIN_MOTION_METRICS'] = str(directory / 'motion.jsonl')
        for argument, variable in (('planner_id', 'TWIN_PLANNER_ID'),
                                   ('smoother', 'TWIN_SMOOTHER'),
                                   ('trajectory_check', 'TWIN_TRAJECTORY_CHECK'),
                                   ('arrival_shadow', 'TWIN_ARRIVAL_SHADOW')):
            value = getattr(args, argument)
            if value is not None:
                env[variable] = str(value)
        record = {'session': marker, 'domain': env['ROS_DOMAIN_ID'], 'gui': args.gui,
                  'parcel_count':batch.count,'parcel_pitch':batch.pitch,
                  'quick_start': args.quick_start, 'before_checks': not args.quick_start, 'passed': False}
        record['motion_experiment'] = {key: env[key] for key in (
            'TWIN_PLANNER_ID', 'TWIN_PLANNING_TIME', 'TWIN_PLANNING_ATTEMPTS',
            'TWIN_SMOOTHER', 'TWIN_TRAJECTORY_CHECK', 'TWIN_ARRIVAL_SHADOW') if key in env}
        def command(cmd, filename, timeout=120):
            with (directory / filename).open('w') as out:
                rc = subprocess.run(cmd, env=env, cwd=ROOT, stdout=out,
                                    stderr=subprocess.STDOUT, timeout=timeout).returncode
            if rc:
                raise RuntimeError(f'{filename}: exit {rc}')
        try:
            launched_at = time.monotonic()
            with (directory / 'bringup.log').open('w') as out:
                launch = subprocess.Popen(['ros2', 'launch', 'twin_action_demo',
                    'industrial_parcel_cell_bringup.launch.py', f'headless:={str(not args.gui).lower()}',
                    f'indexed:={str(args.mode == "indexed").lower()}',
                    f'continuous:={str(args.mode == "continuous").lower()}',
                    f'parcel_count:={batch.count}',f'parcel_pitch:={batch.pitch}',
                    f'grasp_sensors:={str(args.grasp_probe or args.grasp_mode=="confirmed").lower()}'], env=env, cwd=ROOT,
                    stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
            record['launch_pid'] = launch.pid
            ready = False
            for _ in range(50):
                if launch.poll() is not None:
                    raise RuntimeError('Bringup exited before readiness')
                try:
                    status = subprocess.run(['ros2', 'control', 'list_controllers'],
                        capture_output=True, text=True, env=env, timeout=8)
                    if status.stdout.count('active') >= 3:
                        ready = True
                        break
                except subprocess.TimeoutExpired:
                    pass
                time.sleep(1)
            if not ready:
                raise RuntimeError('Controllers not ready')
            if not args.quick_start:
                time.sleep(3)
            with (directory / 'perception_observer.log').open('w') as observer_log:
                observer = [sys.executable, 'tools/check_detections_parcel_cell.py',
                            '--watch-output', str(directory / 'perception_snapshot.json')]
                if args.mode == 'continuous':
                    observer.append('--continuous')
                subprocess.Popen(observer, env=env, stdout=observer_log, stderr=subprocess.STDOUT)
            if not args.quick_start:
                check = [sys.executable, 'tools/check_parcel_cell.py', '--output',
                         str(directory / 'geometry.json'), '--capture', str(directory)]
                if args.mode == 'indexed':
                    check.append('--indexed')
                if args.mode == 'continuous':
                    check.append('--continuous')
                command(check, 'geometry.log')
                perception = [sys.executable, 'tools/check_detections_parcel_cell.py', '--output',
                              str(directory / 'detections_before.json'), '--snapshot',
                              str(directory / 'perception_snapshot.json')]
                if args.mode in ('indexed', 'continuous'):
                    perception.append('--indexed')
                if args.mode == 'continuous':
                    perception.append('--continuous')
                command(perception, 'detections_before.log')
            if args.mode == 'transport':
                command([sys.executable, 'tools/probe_parcel_transport.py', '--output',
                         str(directory / 'transport_probe.json')], 'transport_probe.log')
            if args.mode == 'faults':
                command([sys.executable, 'tools/probe_parcel_faults.py', '--output',
                         str(directory / 'faults.json')], 'faults.log')
            if args.mode in ('stationary', 'indexed', 'continuous'):
                if args.grasp_probe or args.grasp_mode=='confirmed':
                    with (directory/'grasp_observer.log').open('w') as out:
                        subprocess.Popen([sys.executable,'tools/observe_parcel_grasp.py','--output',
                                          str(directory/'grasp_evidence.jsonl')],env=env,stdout=out,stderr=subprocess.STDOUT)
                with (directory / 'observer.log').open('w') as observer_log:
                    observer = [sys.executable, 'tools/observe_parcel_transport.py',
                                '--output', str(directory / 'transport.json')]
                    if args.mode == 'continuous':
                        observer.append('--continuous')
                    subprocess.Popen(observer, env=env,
                        stdout=observer_log, stderr=subprocess.STDOUT)
                task = ['ros2', 'run', 'twin_moveit_scripts', 'parcel_task',
                        '--output', str(directory / 'task.json'),'--parcel-count',str(batch.count),
                        '--parcel-pitch',str(batch.pitch)]
                if args.mode == 'indexed':
                    task.append('--indexed')
                if args.mode == 'continuous':
                    task.append('--continuous')
                    task.extend(['--grasp-mode',args.grasp_mode])
                    if args.belt_speed is not None:
                        task.extend(['--belt-speed',str(args.belt_speed)])
                    if args.grasp_probe:
                        task.append('--grasp-sensors')
                    if args.handoff_probe:
                        task.append('--handoff-probe')
                record['launch_to_task_wall'] = time.monotonic()-launched_at
                with (directory / 'task.log').open('w') as out:
                    rc = subprocess.run(task, env=env, stdout=out,
                                        stderr=subprocess.STDOUT, timeout=1100 if batch.count==3 else 1800).returncode
                record['task_exit'] = rc
                if (directory / 'task.json').exists():
                    record['task_error'] = json.loads((directory / 'task.json').read_text()).get('error')
                command([sys.executable, 'tools/check_parcel_cell.py', '--placed', '--output',
                         str(directory / 'placed.json'), '--capture', str(directory / 'after')],
                        'placed.log')
                after = [sys.executable, 'tools/check_detections_parcel_cell.py', '--placed',
                         '--output', str(directory / 'detections_after.json'), '--snapshot',
                         str(directory / 'perception_snapshot.json')]
                if args.mode == 'continuous':
                    after.append('--continuous')
                command(after, 'detections_after.log')
                result = json.loads((directory / 'task.json').read_text())
                if not result['passed']:
                    raise RuntimeError('Task state machine did not complete')
                if rc not in (0, 245, -11):
                    raise RuntimeError(f'Unexpected task exit {rc}')
                if rc != 0:
                    task_log = (directory / 'task.log').read_text()
                    if ('PARCEL_TASK_COMPLETE' not in task_log or 'Deleting MoveItCpp' not in task_log
                            or task_log.index('PARCEL_TASK_COMPLETE') > task_log.index('Deleting MoveItCpp')):
                        raise RuntimeError('Nonzero exit without known post-completion teardown evidence')
                if args.mode == 'indexed':
                    transport = json.loads((directory / 'transport.json').read_text())
                    if set(transport) != {'parcel_1', 'parcel_2', 'parcel_3'} or not all(
                        value['max_y']-value['initial_y'] >= .20 and value['samples'] > 10
                        for value in transport.values()):
                        raise RuntimeError('Independent conveyor displacement evidence failed')
                if args.mode == 'continuous':
                    command([sys.executable, 'tools/check_continuous_transport.py',
                             '--directory', str(directory)], 'continuous_verdict.log')
                    if args.grasp_mode=='confirmed':
                        command([sys.executable,'tools/check_parcel_grasp.py','--directory',str(directory)],
                                'grasp_verdict.log')
            bad = []
            for name in ('bringup.log', 'task.log'):
                path = directory / name
                if path.exists():
                    for line in path.read_text(errors='replace').splitlines():
                        if any(token in line for token in ('[ERROR]', '[FATAL]', 'Traceback (most recent',
                                                           'Planning FAILED', 'terminate called')):
                            if 'No 3D sensor plugin' not in line:
                                bad.append(line)
            record['unexpected_errors'] = bad
            if bad:
                raise RuntimeError(f'{len(bad)} unexpected log errors')
            record['passed'] = True
        except Exception as exc:
            record['error'] = str(exc)
        finally:
            record['cleanup'] = cleanup(marker)
            (directory / 'result.json').write_text(json.dumps(record, indent=2) + '\n')
        outcomes.append(record)
        print(json.dumps({k: v for k, v in record.items() if k != 'cleanup'}), flush=True)
        if not record['passed']:
            break
    (args.logdir / 'summary.json').write_text(json.dumps(outcomes, indent=2) + '\n')
    return 0 if len(outcomes) == args.runs and all(r['passed'] for r in outcomes) else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('geometry', 'transport', 'faults', 'stationary', 'indexed', 'continuous', 'colour', 'audit'), default='stationary')
    parser.add_argument('--runs', type=int, default=2)
    parser.add_argument('--gui', action='store_true', help='Exercise the interactive Gazebo launch instead of headless rendering')
    parser.add_argument('--quick-start', action='store_true',
                        help='Start sorting as soon as controllers are active; score placement/perception after the batch')
    parser.add_argument('--domain', type=int, default=94)
    parser.add_argument('--grasp-mode', choices=('scheduled','confirmed'), default='scheduled')
    parser.add_argument('--grasp-probe', action='store_true')
    parser.add_argument('--motion-metrics', action='store_true', help='Record per-run planned motion JSONL')
    parser.add_argument('--planner-id', help='Optional parcel-only OMPL planner config')
    parser.add_argument('--smoother', choices=('off', 'stomp'))
    parser.add_argument('--trajectory-check', choices=('off', 'sampled'))
    parser.add_argument('--arrival-shadow', type=lambda p: Path(p).resolve(),
                        help='Explicit experimental simulation arrival profile JSON')
    parser.add_argument('--handoff-probe', action='store_true')
    parser.add_argument('--parcel-count',type=int,choices=(3,7,8,10),default=3)
    parser.add_argument('--parcel-pitch',type=float)
    parser.add_argument('--belt-speed',type=float,help='Conveyor speed (m/s); default is the layout speed')
    parser.add_argument('--logdir', type=Path, required=True)
    args = parser.parse_args()
    try:batch_layout(args.parcel_count,args.parcel_pitch)
    except ValueError as exc:parser.error(str(exc))
    if args.parcel_count!=3 and (args.mode!='continuous' or args.grasp_mode!='confirmed'):
        parser.error('7/8/10-parcel profiles require --mode continuous --grasp-mode confirmed')
    args.logdir = args.logdir.resolve()
    if args.quick_start and args.mode not in ('stationary', 'indexed', 'continuous'):
        parser.error('--quick-start requires a sorting mode')
    if args.mode == 'colour' and any((args.planner_id, args.smoother,
                                      args.trajectory_check, args.arrival_shadow)):
        parser.error('Motion experiment arguments are parcel-only; keep colour regression unmodified')
    if args.runs < 1 or not 1 <= args.domain <= 100 or args.domain + args.runs > 101:
        parser.error('Invalid run count or ROS domain range')
    if args.mode == 'audit':
        sys.exit(audit_finished(args))
    sys.exit(run_colours(args) if args.mode == 'colour' else run(args))

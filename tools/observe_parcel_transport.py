#!/usr/bin/env python3
"""Read-only Gazebo transport evidence, separate from robot vision/control."""
import argparse
import json
from pathlib import Path
import signal
import time
import threading

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.double_pb2 import Double
from gz.transport13 import Node
from twin_moveit_scripts.parcel_layout import active_batch


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--continuous', action='store_true')
    args = parser.parse_args()
    names=frozenset(active_batch().names)
    running = True
    history = {}
    evidence = {'parcels': history, 'belt': [], 'poses': []}
    latest_stamp = 0.0
    sample_stamp = -1.0
    lock = threading.Lock()
    def stop(*_):
        nonlocal running
        running = False
    def callback(msg):
        nonlocal latest_stamp, sample_stamp
        with lock:
            latest_stamp = msg.header.stamp.sec+msg.header.stamp.nsec/1e9
            sample = args.continuous and latest_stamp-sample_stamp >= .1
            for pose in msg.pose:
                if pose.name in names:
                    p = pose.position
                    if sample:
                        evidence['poses'].append([latest_stamp, pose.name, p.x, p.y, p.z])
                    if abs(p.x - .56) < .025 and abs(p.z - .320) < .012:
                        values = history.setdefault(pose.name, {'initial_y': p.y, 'min_y': p.y,
                                                               'max_y': p.y, 'samples': 0})
                        values['min_y'] = min(values['min_y'], p.y)
                        values['max_y'] = max(values['max_y'], p.y)
                        values['samples'] += 1
            if sample:
                sample_stamp = latest_stamp

    def belt(msg):
        with lock:
            evidence['belt'].append([latest_stamp, msg.data])
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node = Node()
    if not node.subscribe(Pose_V, '/world/empty/dynamic_pose/info', callback):
        raise RuntimeError('Cannot subscribe to independent pose evidence')
    if args.continuous and not node.subscribe(Double, '/parcel_cell/belt_speed', belt):
        raise RuntimeError('Cannot subscribe to actual Gazebo belt command')
    while running:
        with lock:
            temporary = args.output.with_suffix('.tmp')
            temporary.write_text(json.dumps(evidence if args.continuous else history, indent=2)+'\n')
            temporary.replace(args.output)
        time.sleep(.5)


if __name__ == '__main__':
    main()

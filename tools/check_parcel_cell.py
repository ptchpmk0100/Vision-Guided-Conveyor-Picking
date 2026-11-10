#!/usr/bin/env python3
"""Independent live Gazebo parcel geometry/placement verdict and image capture."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/twin_moveit_scripts'))
from twin_moveit_scripts.parcel_layout import (PARCEL_NAMES, PARCEL_Z, PICK_X, QUEUE_Y,
    CONTINUOUS_QUEUE_Y, STAGING_Y, inside_tote)
from twin_moveit_scripts.parcel_layout import active_batch,batch_layout,footprints_clear


def _gz_echo(topic, attempts=4):
    # `gz topic -e -n 1` aborts transiently on teardown (observed SIGABRT) even while the
    # topic is healthy, so retry a crashed/empty read before giving up. A genuinely missing
    # or misplaced parcel still shows up once a read succeeds, so this cannot mask a defect.
    last = None
    for attempt in range(attempts):
        try:
            raw = subprocess.run(['gz', 'topic', '-e', '-n', '1', '-t', topic],
                                 capture_output=True, text=True, timeout=15, check=True)
            if raw.stdout.strip():
                return raw.stdout
            last = RuntimeError(f'empty read from {topic}')
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
            last = exc
        time.sleep(0.5)
    raise last


def poses():
    from google.protobuf import text_format
    from gz.msgs10.pose_v_pb2 import Pose_V
    result = {}
    for topic in ('/world/empty/pose/info', '/world/empty/dynamic_pose/info'):
        message = text_format.Parse(_gz_echo(topic), Pose_V())
        for p in message.pose:
            q = p.orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
            pitch = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y-q.z*q.x))))
            result[p.name] = (p.position.x, p.position.y, p.position.z, yaw, roll, pitch)
    return result


def verdict(placed=False, indexed=False, continuous=False,parcel_count=None,parcel_pitch=None):
    batch=active_batch() if parcel_count is None else batch_layout(parcel_count,parcel_pitch)
    samples = [poses()]
    time.sleep(1.0)
    samples.append(poses())
    ys = batch.queue_y if continuous else (QUEUE_Y if indexed else STAGING_Y)
    checks = []
    for i, name in enumerate(batch.names):
        if not all(name in sample for sample in samples):
            checks.append({'name': name, 'passed': False, 'error': 'missing entity'})
            continue
        x, y, z, yaw, roll, pitch = samples[-1][name]
        stable = math.dist(samples[0][name][:3], samples[1][name][:3]) < .003
        upright = abs(roll) < .03 and abs(pitch) < .03
        z_ok = abs(z - PARCEL_Z) < .008
        xy_ok = batch.contains(x,y,yaw) if placed else math.hypot(x-PICK_X, y-ys[i]) < .020
        checks.append({'name': name, 'position': [x, y, z], 'yaw': yaw,
                       'stable': stable, 'height_ok': z_ok, 'footprint_ok': xy_ok,
                       'roll_pitch': [roll, pitch], 'upright': upright,
                       'passed': stable and z_ok and xy_ok and upright})
    found = sorted(n for n in samples[-1] if n.startswith('parcel_') and n[7:].isdigit())
    exact = found == sorted(batch.names)
    if placed:
        if batch.extended:
            used=set()
            for row in checks:
                if 'position' not in row:continue
                distance,slot=min((math.dist(row['position'][:2],xy),i) for i,xy in enumerate(batch.slots))
                row.update(slot=slot,slot_error_m=distance)
                row['passed'] &= distance<=.015 and slot not in used
                used.add(slot)
        # Disallow two cartons being counted in the same occupied footprint.
        for i, first in enumerate(checks):
            for other in checks[i+1:]:
                if 'position' in first and 'position' in other:
                    conflict=(not footprints_clear((*first['position'][:2],first['yaw']),
                              (*other['position'][:2],other['yaw'])) if batch.extended else
                              math.dist(first['position'][:2], other['position'][:2]) < .055)
                    if conflict:
                        first['passed'] = other['passed'] = False
    return {'mode': 'placed' if placed else 'geometry', 'checks': checks,
            'exact_entities': exact, 'passed': exact and all(c['passed'] for c in checks)}


def capture(directory):
    import cv2
    import numpy as np
    from gz.msgs10.image_pb2 import Image
    from gz.transport13 import Node
    # Record actual sensor frames directly from Gazebo transport. Functional
    # ROS depth/detection interfaces are checked independently by the detector
    # checker; this avoids a second ROS image reader's reconnection/discovery race.
    node = Node()
    received = {}
    def callback(msg, name):
        if name in received:
            return
        if msg.pixel_format_type not in (3, 4, 5, 8):
            return
        channels = 3 if msg.pixel_format_type in (3, 8) else 4
        data = np.ndarray((msg.height, msg.width, channels), dtype=np.uint8,
                          buffer=msg.data, strides=(msg.step, channels, 1))
        if msg.pixel_format_type in (3, 4):
            data = cv2.cvtColor(data, cv2.COLOR_RGB2BGR if channels == 3 else cv2.COLOR_RGBA2BGR)
        elif msg.pixel_format_type == 5:
            data = cv2.cvtColor(data, cv2.COLOR_BGRA2BGR)
        path = directory / f'{name}.png'
        if not cv2.imwrite(str(path), data):
            raise RuntimeError(f'Failed to capture {path}')
        received[name] = str(path)
    topics = [('overview', '/parcel_cell/overview'), ('side', '/parcel_cell/side'),
              ('rgbd', '/overhead_camera/image')]
    for name, topic in topics:
        if not node.subscribe(Image, topic, lambda msg, n=name: callback(msg, n)):
            raise RuntimeError(f'Could not subscribe to camera {topic}')
    deadline = time.monotonic() + 30
    while len(received) < len(topics) and time.monotonic() < deadline:
        time.sleep(.1)
    for _, topic in topics:
        node.unsubscribe(topic)
    if len(received) != 3:
        raise RuntimeError(f'Missing camera evidence: {received}')
    return received


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--placed', action='store_true')
    parser.add_argument('--indexed', action='store_true')
    parser.add_argument('--continuous', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--capture', type=Path)
    parser.add_argument('--parcel-count',type=int,choices=(3,7,8,10))
    parser.add_argument('--parcel-pitch',type=float)
    args = parser.parse_args()
    result = verdict(args.placed, args.indexed, args.continuous,args.parcel_count,args.parcel_pitch)
    if args.output:
        args.output.write_text(json.dumps(result, indent=2) + '\n')
    if args.capture:
        args.capture.mkdir(parents=True, exist_ok=True)
        result['images'] = capture(args.capture)
    if args.output:
        args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())

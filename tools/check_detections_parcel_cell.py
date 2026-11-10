#!/usr/bin/env python3
"""Score the parcel detector against live Gazebo poses, never detector log status."""
import argparse
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from vision_msgs.msg import Detection3DArray

from check_parcel_cell import poses
from twin_moveit_scripts.parcel_layout import active_batch


def regions_for(continuous):
    regions = [('pick', '/parcel_cell/detections'), ('tote', '/parcel_cell/tote_detections')]
    if continuous:
        regions.append(('moving', '/parcel_cell/moving_detections'))
    return regions


def observation(message):
    points = []
    for d in message.detections:
        p = d.bbox.center.position
        points.append({'class': d.results[0].hypothesis.class_id if d.results else '',
                       'xyz': [p.x, p.y, p.z]})
    return dict(frame=message.header.frame_id,
                stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9,
                received_wall=time.time(), points=points)


def watch(args):
    """Keep the independent ROS reader connected before, during and after motion."""
    rclpy.init()
    node = rclpy.create_node('independent_parcel_observer')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    messages = {}
    subscriptions = [node.create_subscription(Detection3DArray, topic,
        lambda msg, n=name: messages.update({n: observation(msg)}), 10)
        for name, topic in regions_for(args.continuous)]
    next_write = 0.
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic() < next_write:
                continue
            next_write = time.monotonic()+.1
            data = {'written_wall': time.time(),
                    'sim_time': node.get_clock().now().nanoseconds/1e9, 'regions': messages}
            temp = args.watch_output.with_suffix('.tmp')
            temp.write_text(json.dumps(data)+'\n')
            temp.replace(args.watch_output)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


def snapshot_messages(data):
    """Require genuinely fresh independent messages, never cached success evidence."""
    if time.time()-data['written_wall'] > .7:
        return {}
    return {name: value for name, value in data['regions'].items()
            if time.time()-value['received_wall'] <= 1.
            and value['frame'] == 'world' and -.05 <= data['sim_time']-value['stamp'] <= .7}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--placed', action='store_true')
    parser.add_argument('--indexed', action='store_true')
    parser.add_argument('--continuous', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--watch-output', type=Path)
    parser.add_argument('--snapshot', type=Path)
    args = parser.parse_args()
    if args.watch_output:
        return watch(args)
    if args.output is None:
        parser.error('--output is required for a verdict')
    batch=active_batch()
    truth = {name: p[:3] for name, p in poses().items() if name in batch.names}
    messages = {}
    regions = regions_for(args.continuous)
    node = None
    if args.snapshot:
        messages = snapshot_messages(json.loads(args.snapshot.read_text()))
    else:
        rclpy.init()
        node = rclpy.create_node('independent_parcel_detection_check')
        subscriptions = [node.create_subscription(Detection3DArray, topic,
                         lambda msg, n=name: messages.update({n: observation(msg)}), qos_profile_sensor_data)
                         for name, topic in regions]
        deadline = time.monotonic() + 15
        while len(messages) < len(subscriptions) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.2)
    result = {'passed': False, 'regions': {}}
    for name, _ in regions:
        want = truth if ((name == 'tote' and args.placed) or
                         (name == 'pick' and not args.placed and not args.indexed)) else {}
        if name == 'moving':
            want = {entity: xyz for entity, xyz in truth.items()
                    if .505 <= xyz[0] <= .615 and -.4 <= xyz[1] <= .18}
        message = messages.get(name)
        dets = message['points'] if message is not None else []
        used = set()
        matches = []
        for entity, target in want.items():
            candidates = []
            for index, d in enumerate(dets):
                if index not in used and d['class'] == 'parcel_box':
                    candidates.append((math.dist(d['xyz'], target), index))
            distance, index = min(candidates, default=(float('inf'), -1))
            if distance <= .010:
                used.add(index)
            matches.append({'entity': entity, 'error_m': distance if math.isfinite(distance) else None,
                            'passed': distance <= .010})
        result['regions'][name] = {'received': message is not None, 'expected': len(want),
            'detected': len(dets), 'matches': matches,
            'passed': message is not None and len(dets) == len(want) == len(used)}
    result['passed'] = len(truth) == batch.count and all(v['passed'] for v in result['regions'].values())
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    if node is not None:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())

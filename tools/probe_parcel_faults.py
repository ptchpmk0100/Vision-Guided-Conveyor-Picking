#!/usr/bin/env python3
"""Inject sensor faults on isolated test topics; assert the real feed guard stops."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from std_msgs.msg import Float64
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose

from twin_moveit_scripts.parcel_task import Observations


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('independent_parcel_fault_probe')
    observations = Observations(node, '/parcel_cell/fault_input', '/parcel_cell/fault_tote')
    publisher = node.create_publisher(Detection3DArray, '/parcel_cell/fault_input', 10)
    tote_publisher = node.create_publisher(Detection3DArray, '/parcel_cell/fault_tote', 10)
    speed = [None]
    node.create_subscription(Float64, '/parcel_cell/belt_speed', lambda msg: speed.__setitem__(0, msg.data), 10)
    mode = ['empty']
    def publish():
        msg = Detection3DArray()
        msg.header.frame_id = 'world'
        msg.header.stamp = node.get_clock().now().to_msg()
        if mode[0] == 'stale':
            msg.header.stamp.sec -= 2
        if mode[0] in ('outside', 'nan', 'class', 'multiple', 'full'):
            for _ in range(2 if mode[0] == 'multiple' else 1):
                det = Detection3D()
                det.bbox.center.position.x = .9 if mode[0] == 'outside' else (.56 if mode[0] != 'nan' else float('nan'))
                det.bbox.center.position.y = -.1
                det.bbox.center.position.z = .32
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = 'unknown' if mode[0] == 'class' else 'parcel_box'
                det.results.append(hyp)
                msg.detections.append(det)
        (tote_publisher if mode[0] == 'full' else publisher).publish(msg)
    timer = node.create_timer(.05, publish)
    # DDS discovery may take longer than the first short injected-fault case.
    # Require an actual guard observation before asserting its stop behaviour.
    deadline = time.monotonic() + 8
    while speed[0] is None and time.monotonic() < deadline:
        observations.poll(.1)
    if speed[0] is None:
        raise RuntimeError('Feed guard discovery failed before fault injection')
    checks = []
    for case in ('empty', 'stale', 'outside', 'nan', 'class', 'multiple', 'full'):
        mode[0] = case
        observations.message = None
        observations.poll(.3)
        error = None
        try:
            if case == 'full':
                observations.require_empty_tote()
            else:
                observations.feed_one(timeout=2.0 if case == 'stale' else .8)
        except RuntimeError as exc:
            error = str(exc)
        observations.poll(.6)
        checks.append({'fault': case, 'error': error, 'belt_speed_after': speed[0],
                       'passed': error is not None and speed[0] == 0.0})
    node.destroy_timer(timer)
    result = {'checks': checks, 'passed': all(c['passed'] for c in checks)}
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

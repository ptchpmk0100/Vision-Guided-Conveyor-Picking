#!/usr/bin/env python3
"""Live ground-truth check of /parcel_cell/condition against Gazebo while the sim runs (SIGINT/SIGTERM -> write result).

Each published classification is matched to the nearest Gazebo parcel (xy < 4 cm); the true condition comes from the
spawn-time variant manifest. Simulation ground truth only: the control path never reads it.
"""
import argparse
import collections
import json
import math
import signal
import time

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray


class Check(Node):
    def __init__(self, truth, topic):
        super().__init__('condition_live_check')
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GzNode
        self.truth, self.pose, self.rows = truth, {}, []
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, '/world/empty/dynamic_pose/info', self.on_pose)
        self.create_subscription(Detection3DArray, topic, self.on_msg, 10)
        self.first = self.last = None
        self.msgs = 0

    def on_pose(self, msg):
        for p in msg.pose:
            if p.name in self.truth:
                self.pose[p.name] = (p.position.x, p.position.y)

    def on_msg(self, msg):
        now = time.monotonic()
        self.first = self.first or now
        self.last = now
        self.msgs += 1
        poses = dict(self.pose)
        for det in msg.detections:
            c = det.bbox.center.position
            name, d = min(((n, math.hypot(c.x - p[0], c.y - p[1])) for n, p in poses.items()), key=lambda t: t[1],
                          default=(None, 9))
            if name and d < 0.04:
                self.rows.append((name, self.truth[name], det.results[0].hypothesis.class_id,
                                  det.results[0].hypothesis.score))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--manifest', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--topic', default='/parcel_cell/condition')
    args = ap.parse_args()
    truth = {p['name']: p['condition'] for p in json.load(open(args.manifest))['parcels']}
    rclpy.init()
    node = Check(truth, args.topic)
    stop = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.append(1))
    while not stop:
        rclpy.spin_once(node, timeout_sec=0.2)
    tally = collections.Counter((t, p) for _, t, p, _ in node.rows)
    per = collections.defaultdict(collections.Counter)
    for n, t, p, _ in node.rows:
        per[n][p] += 1
    parcels = {n: {'true': truth[n], 'votes': dict(c), 'majority': c.most_common(1)[0][0]} for n, c in per.items()}
    result = {'messages': node.msgs, 'classified_detections': len(node.rows),
              'per_detection': {f'{t}->{p}': v for (t, p), v in sorted(tally.items())},
              'parcels_seen': len(parcels),
              'parcels_majority_correct': sum(v['true'] == v['majority'] for v in parcels.values()),
              'parcels': parcels,
              'mean_confidence': sum(r[3] for r in node.rows) / max(1, len(node.rows))}
    json.dump(result, open(args.out, 'w'), indent=1)
    print(json.dumps({k: v for k, v in result.items() if k != 'parcels'}))
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

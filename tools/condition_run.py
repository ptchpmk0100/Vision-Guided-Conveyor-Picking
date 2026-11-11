#!/usr/bin/env python3
"""One dataset recording against an ALREADY RUNNING parcel sim (see tools/condition_collect.sh).

Republishes Gazebo parcel poses (simulation-only ground truth) at 20 sim-Hz on /gt/parcel_poses, starts
`ros2 bag record`, runs the belt at a fixed speed until every parcel has left the camera view, then stops.
Labels never come from the depth detector.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage

# raw depth is NOT bagged (halves the I/O at a low sim real-time factor); the live depth detector's output is.
BAG_TOPICS = ['/overhead_camera/image', '/overhead_camera/camera_info',
              '/clock', '/tf', '/tf_static', '/joint_states', '/gt/parcel_poses',
              '/parcel_cell/moving_detections', '/parcel_cell/detections', '/parcel_cell/belt_speed']


class Run(Node):
    def __init__(self, args):
        super().__init__('condition_run', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.args = args
        self.names = {f'parcel_{i}' for i in range(1, args.count + 1)}
        self.latest = {}
        self.stamp_wall = time.monotonic()
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GzNode
        self.gz = GzNode()          # names are lost by ros_gz_bridge, so subscribe to Gazebo directly
        self.gz.subscribe(Pose_V, '/world/empty/dynamic_pose/info', self.pose)
        self.gt_pub = self.create_publisher(TFMessage, '/gt/parcel_poses', 10)
        self.feed_pub = self.create_publisher(Float64, '/parcel_cell/feed_command', 10)
        self.speed = 0.0
        self.create_timer(0.05, self.publish_gt)
        # belt deadman needs a command every <0.4 WALL seconds, whatever the sim speed is
        self.create_timer(0.1, lambda: self.feed_pub.publish(Float64(data=self.speed)),
                          clock=Clock(clock_type=ClockType.SYSTEM_TIME))

    def pose(self, msg):                       # gz-transport thread
        for p in msg.pose:
            if p.name in self.names:
                self.latest[p.name] = (p.position.x, p.position.y, p.position.z, p.orientation.x,
                                       p.orientation.y, p.orientation.z, p.orientation.w)

    def publish_gt(self):
        if len(self.latest) < len(self.names):
            return
        out = TFMessage()
        now = self.get_clock().now().to_msg()
        for name, v in sorted(dict(self.latest).items()):
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'world'
            t.child_frame_id = name
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = v[:3]
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = v[3:]
            out.transforms.append(t)
        self.gt_pub.publish(out)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def ys(self):
        return {n: v[1] for n, v in dict(self.latest).items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--speed', type=float, required=True)
    ap.add_argument('--count', type=int, default=10)
    ap.add_argument('--y-end', type=float, default=0.15, help='stop when every parcel is beyond this y (the belt queues parcels at an end stop near y=0.74)')
    ap.add_argument('--wall-timeout', type=float, default=420.0)
    ap.add_argument('--no-bag', action='store_true')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rclpy.init()
    node = Run(args)
    summary = {'speed': args.speed, 'count': args.count, 'ok': False}
    bag = None
    try:
        t0 = time.monotonic()
        while len(node.latest) < args.count:
            node.spin_for(0.5)
            if time.monotonic() - t0 > 90:
                raise RuntimeError(f'only saw {sorted(node.latest)} parcels in 90 s')
        node.spin_for(2.0)
        start_y = node.ys()
        if not args.no_bag:
            bag_dir = os.path.join(args.out, 'bag')
            bag = subprocess.Popen(['ros2', 'bag', 'record', '--use-sim-time', '-o', bag_dir,
                                    *BAG_TOPICS], stdout=open(os.path.join(args.out, 'bag.log'), 'w'),
                                   stderr=subprocess.STDOUT, start_new_session=True)
            node.spin_for(5.0)
        sim0 = node.get_clock().now().nanoseconds / 1e9
        node.speed = args.speed
        run0 = time.monotonic()
        while True:
            node.spin_for(0.5)
            ys = node.ys()
            if min(ys.values()) > args.y_end:
                summary['ok'] = True
                break
            if time.monotonic() - run0 > args.wall_timeout:
                summary['error'] = 'wall timeout'
                break
        node.speed = 0.0
        node.spin_for(1.0)
        summary.update(sim_seconds=round(node.get_clock().now().nanoseconds / 1e9 - sim0, 1),
                       wall_seconds=round(time.monotonic() - run0, 1),
                       start_y={k: round(v, 3) for k, v in sorted(start_y.items())},
                       end_y={k: round(v, 3) for k, v in sorted(node.ys().items())})
        z = {n: v[2] for n, v in dict(node.latest).items()}
        summary['fell'] = sorted(n for n, v in z.items() if v < 0.30)
    except Exception as exc:  # noqa: BLE001 - recorded into the summary
        summary['error'] = repr(exc)
    finally:
        node.speed = 0.0
        node.spin_for(0.5)
        if bag is not None:
            os.killpg(bag.pid, signal.SIGINT)
            try:
                bag.wait(timeout=60)
            except subprocess.TimeoutExpired:
                os.killpg(bag.pid, signal.SIGKILL)
        with open(os.path.join(args.out, 'run_summary.json'), 'w') as handle:
            json.dump(summary, handle, indent=1)
        print(json.dumps(summary))
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if summary['ok'] else 1)


if __name__ == '__main__':
    main()

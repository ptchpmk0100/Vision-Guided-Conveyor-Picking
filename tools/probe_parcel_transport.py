#!/usr/bin/env python3
"""Independent conveyor and command-loss stop test on live dynamic parcels."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from std_msgs.msg import Float64

from check_parcel_cell import poses


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('independent_conveyor_probe')
    publisher = node.create_publisher(Float64, '/parcel_cell/feed_command', 10)
    start = poses()
    # Deliberately lose the command stream after this interval. The guard,
    # not this probe, must bring the belt to rest.
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        publisher.publish(Float64(data=.035))
        rclpy.spin_once(node, timeout_sec=.04)
    time.sleep(1.2)
    stopped = poses()
    time.sleep(1.0)
    later = poses()
    checks = []
    for name in ('parcel_1', 'parcel_2', 'parcel_3'):
        delta = stopped[name][1] - start[name][1]
        drift = abs(later[name][1] - stopped[name][1])
        checks.append(dict(name=name, delta_y=delta, drift_after_command_loss=drift,
                           passed=delta > .012 and drift < .003))
    # Invalid commands must also inhibit transport.
    for value in (float('nan'), -.1, .5):
        publisher.publish(Float64(data=value))
        rclpy.spin_once(node, timeout_sec=.15)
    time.sleep(.7)
    invalid = poses()
    invalid_ok = all(abs(invalid[n][1]-later[n][1]) < .003 for n in ('parcel_1', 'parcel_2', 'parcel_3'))
    publisher.publish(Float64(data=0.0))
    result = dict(checks=checks, invalid_commands_stopped=invalid_ok,
                  passed=invalid_ok and all(c['passed'] for c in checks))
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

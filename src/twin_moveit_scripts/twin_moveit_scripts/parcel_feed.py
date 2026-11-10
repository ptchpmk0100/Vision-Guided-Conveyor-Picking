"""Belt command deadman: missing/nonfinite/out-of-range demand stops the track."""
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from twin_moveit_scripts.parcel_layout import BELT_TOPIC


class FeedGuard(Node):
    def __init__(self):
        super().__init__('parcel_feed_guard')
        self.speed = 0.0
        self.max_speed = float(self.declare_parameter('max_speed', .04).value)
        if not math.isfinite(self.max_speed) or not 0 < self.max_speed <= .08:
            raise ValueError('max_speed must be finite and in (0, .08] m/s')
        self.received = 0.0
        self.pub = self.create_publisher(Float64, BELT_TOPIC, 10)
        self.create_subscription(Float64, '/parcel_cell/feed_command', self.command, 10)
        self.create_timer(.05, self.tick)

    def command(self, msg):
        self.speed = msg.data if math.isfinite(msg.data) and 0 <= msg.data <= self.max_speed else 0.0
        self.received = time.monotonic()

    def tick(self):
        self.pub.publish(Float64(data=self.speed if time.monotonic()-self.received < .4 else 0.0))


def main():
    rclpy.init()
    node = FeedGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Float64(data=0.0))
        node.destroy_node()
        rclpy.try_shutdown()

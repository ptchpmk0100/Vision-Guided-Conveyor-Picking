"""
finger_state_publisher.py

Publishes state for ONLY the two gripper finger joints onto /joint_states.

Why: move_group's model (full URDF) has 9 joints; joint_state_broadcaster only
reports the 7 arm joints in <ros2_control>. The 2 fingers have no state source,
so move_group blocks on "missing panda_finger_jointN". This fills that gap.

Ownership: broadcaster owns the 7 arm joints; this node owns ONLY the 2 fingers.
No joint is published by both, so merging on /joint_states is conflict-free.

panda_finger_joint2 mimics joint1 (mult 1.0), so both carry the same value.
0.02 m is mid-range within the [0.0, 0.04] prismatic limit.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FingerStatePublisher(Node):
    def __init__(self):
        super().__init__("finger_state_publisher")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(1.0 / 30.0, self.tick)
        self.position = 0.02

    def tick(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["panda_finger_joint1", "panda_finger_joint2"]
        msg.position = [self.position, self.position]
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FingerStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

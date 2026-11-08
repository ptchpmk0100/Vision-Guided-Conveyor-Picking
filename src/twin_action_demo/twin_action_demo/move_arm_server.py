import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from twin_interfaces.action import MoveArm

# The 7 arm joints, in goal order. Index i in target_angles maps to JOINTS[i].
JOINTS = [f'panda_joint{i}' for i in range(1, 8)]


class MoveArmServer(Node):
    def __init__(self):
        super().__init__('move_arm_server')

        # Latest measured angles, keyed by joint name. Empty until first joint_state.
        self.latest = {}

        cb_group = ReentrantCallbackGroup()

        # SEVEN publishers — one per joint's cmd_pos topic. Dict keyed by joint name.
        self.cmd_pubs = {
            j: self.create_publisher(Float64, f'/{j}/cmd_pos', 10)
            for j in JOINTS
        }

        # ONE subscriber — joint_state carries ALL joints at once.
        self.create_subscription(
            JointState,
            '/world/empty/model/panda/joint_state',
            self.joint_state_callback,
            10,
            callback_group=cb_group,
        )

        self._server = ActionServer(
            self,
            MoveArm,
            'move_arm',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=cb_group,
        )
        self.get_logger().info('MoveArm server ready.')

    def joint_state_callback(self, msg):
        # Store every joint we care about, by name (order in msg is not guaranteed).
        for j in JOINTS:
            if j in msg.name:
                self.latest[j] = msg.position[msg.name.index(j)]

    def goal_callback(self, goal_request):
        targets = goal_request.target_angles
        # Validate: exactly 7 values, each within +/- 3.14.
        if len(targets) != 7:
            self.get_logger().warn(
                f'Rejecting goal: got {len(targets)} angles, need 7.')
            return GoalResponse.REJECT
        if any(abs(t) > 3.14 for t in targets):
            self.get_logger().warn('Rejecting goal: an angle is out of bounds.')
            return GoalResponse.REJECT
        self.get_logger().info(f'Accepting goal: {[round(t, 2) for t in targets]}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested — accepting.')
        return CancelResponse.ACCEPT

    def _have_all_measurements(self):
        return all(j in self.latest for j in JOINTS)

    def execute_callback(self, goal_handle):
        targets = list(goal_handle.request.target_angles)
        feedback = MoveArm.Feedback()

        # Wait until we've measured all 7 joints at least once.
        while not self._have_all_measurements():
            self.get_logger().info('Waiting for first full joint_state...')
            time.sleep(0.2)

        # 1) SEND ALL 7 COMMANDS ONCE. The PID controllers do the motion.
        for j, t in zip(JOINTS, targets):
            msg = Float64()
            msg.data = t
            self.cmd_pubs[j].publish(msg)
        self.get_logger().info('Commanded all 7 joints, waiting for arrival...')

        # 2) POLL until EVERY joint is within tolerance.
        tolerance = 0.05
        timeout = 15.0
        start = time.time()

        def errors():
            return [abs(self.latest[j] - t) for j, t in zip(JOINTS, targets)]

        while max(errors()) > tolerance:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Motion canceled.')
                result = MoveArm.Result()
                result.success = False
                result.final_angles = [self.latest[j] for j in JOINTS]
                return result

            if time.time() - start > timeout:
                self.get_logger().warn('Timed out before all joints reached target.')
                goal_handle.abort()
                result = MoveArm.Result()
                result.success = False
                result.final_angles = [self.latest[j] for j in JOINTS]
                return result

            feedback.current_angles = [self.latest[j] for j in JOINTS]
            feedback.remaining = errors()
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)

        goal_handle.succeed()
        result = MoveArm.Result()
        result.success = True
        result.final_angles = [self.latest[j] for j in JOINTS]
        self.get_logger().info('All 7 joints reached target.')
        return result


def main():
    rclpy.init()
    node = MoveArmServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
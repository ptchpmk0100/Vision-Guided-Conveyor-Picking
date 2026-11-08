#----------------------Original single-threaded version----------------------#

# import time
# import rclpy
# from rclpy.action import ActionServer, CancelResponse, GoalResponse
# from rclpy.node import Node

# from twin_interfaces.action import MoveJoint


# class MoveJointServer(Node):
#     def __init__(self):
#         super().__init__('move_joint_server')
#         self.current_angle = 0.0
#         self._server = ActionServer(
#             self,
#             MoveJoint,
#             'move_joint',
#             goal_callback=self.goal_callback,
#             cancel_callback=self.cancel_callback,
#             execute_callback=self.execute_callback,
#         )
#         self.get_logger().info('MoveJoint server ready.')

#     def goal_callback(self, goal_request):
#         target = goal_request.target_angle
#         if abs(target) > 3.14:
#             self.get_logger().warn(f'Rejecting target {target:.2f} (out of bounds).')
#             return GoalResponse.REJECT
#         self.get_logger().info(f'Accepting target {target:.2f}.')
#         return GoalResponse.ACCEPT

#     def cancel_callback(self, goal_handle):
#         self.get_logger().info('Cancel requested — accepting.')
#         return CancelResponse.ACCEPT

#     def execute_callback(self, goal_handle):
#         target = goal_handle.request.target_angle
#         feedback = MoveJoint.Feedback()
#         step = 0.05 if target >= self.current_angle else -0.05

#         while abs(self.current_angle - target) > abs(step):
#             if goal_handle.is_cancel_requested:
#                 goal_handle.canceled()
#                 self.get_logger().info('Motion canceled.')
#                 result = MoveJoint.Result()
#                 result.success = False
#                 result.final_angle = self.current_angle
#                 return result

#             self.current_angle += step
#             feedback.current_angle = self.current_angle
#             feedback.remaining = abs(target - self.current_angle)
#             goal_handle.publish_feedback(feedback)
#             time.sleep(0.1)

#         self.current_angle = target
#         goal_handle.succeed()
#         result = MoveJoint.Result()
#         result.success = True
#         result.final_angle = self.current_angle
#         self.get_logger().info(f'Reached {target:.2f}.')
#         return result


# def main():
#     rclpy.init()
#     node = MoveJointServer()
#     rclpy.spin(node)
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()




# ---------------------- Multithreaded version ----------------------#
import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from twin_interfaces.action import MoveJoint


class MoveJointServer(Node):
    def __init__(self):
        super().__init__('move_joint_server')

        # Latest measured angle from Gazebo. None until the first joint_state arrives.
        self.latest_position = None

        # One callback group so the action and the subscriber can run concurrently
        # on the MultiThreadedExecutor (fixes the single-thread deadlock).
        cb_group = ReentrantCallbackGroup()

        # NEW: publisher — sends the target angle into the bridge -> Gazebo controller.
        self.cmd_pub = self.create_publisher(
            Float64,
            '/model/one_joint_arm/joint/joint1/cmd_pos',
            10,
        )

        # NEW: subscriber — reads the REAL measured joint angle back from Gazebo.
        self.create_subscription(
            JointState,
            '/world/empty/model/one_joint_arm/joint_state',
            self.joint_state_callback,
            10,
            callback_group=cb_group,
        )

        self._server = ActionServer(
            self,
            MoveJoint,
            'move_joint',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=cb_group,
        )
        self.get_logger().info('MoveJoint server ready.')

    def joint_state_callback(self, msg):
        # msg.name is a list; find joint1's index and store its position.
        if 'joint1' in msg.name:
            i = msg.name.index('joint1')
            self.latest_position = msg.position[i]

    def goal_callback(self, goal_request):
        target = goal_request.target_angle
        if abs(target) > 3.14:
            self.get_logger().warn(f'Rejecting target {target:.2f} (out of bounds).')
            return GoalResponse.REJECT
        self.get_logger().info(f'Accepting target {target:.2f}.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested — accepting.')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        target = goal_handle.request.target_angle
        feedback = MoveJoint.Feedback()

        # Wait until we have at least one real measurement before commanding.
        while self.latest_position is None:
            self.get_logger().info('Waiting for first joint_state...')
            time.sleep(0.2)

        # 1) SEND THE COMMAND ONCE. The PID controller in Gazebo does the motion.
        cmd = Float64()
        cmd.data = target
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'Commanded {target:.2f}, waiting for arrival...')

        # 2) POLL the real measured angle until it's within tolerance of the target.
        tolerance = 0.05
        timeout = 10.0            # give up after 10 s so we never hang forever
        start = time.time()

        while abs(self.latest_position - target) > tolerance:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Motion canceled.')
                result = MoveJoint.Result()
                result.success = False
                result.final_angle = self.latest_position
                return result

            if time.time() - start > timeout:
                self.get_logger().warn('Timed out before reaching target.')
                goal_handle.abort()
                result = MoveJoint.Result()
                result.success = False
                result.final_angle = self.latest_position
                return result

            # Publish the REAL measured angle as feedback.
            feedback.current_angle = self.latest_position
            feedback.remaining = abs(target - self.latest_position)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)

        goal_handle.succeed()
        result = MoveJoint.Result()
        result.success = True
        result.final_angle = self.latest_position
        self.get_logger().info(f'Reached {self.latest_position:.2f}.')
        return result


def main():
    rclpy.init()
    node = MoveJointServer()
    executor = MultiThreadedExecutor()   # NEW: multi-threaded so callbacks run concurrently
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()





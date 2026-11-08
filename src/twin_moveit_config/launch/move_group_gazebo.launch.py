"""
move_group_gazebo.launch.py

Launches ONLY the move_group node, wired to the EXISTING Gazebo control stack.
  - Forces use_sim_time:=true (our twin runs on /clock; wall-clock move_group desyncs
    TF + trajectory timestamps and execution fails). The stock generator gives no hook
    for this, so we build the node ourselves and append {"use_sim_time": True} last
    (ROS 2 params are last-wins on merge).
  - move_group ALONE. Gazebo bringup already owns robot_state_publisher, the
    controller_manager, arm_controller, and joint_state_broadcaster. We do NOT spawn
    controllers or rsp here (that would double-spawn / conflict).

Run AFTER panda_bringup.launch.py is up (controllers must already be active).
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # URDF resolved via .setup_assistant -> package twin_description,
    # urdf/panda.urdf (the same world-anchored URDF Gazebo simulates).
    moveit_config = MoveItConfigsBuilder(
        "panda", package_name="twin_moveit_config"
    ).to_moveit_configs()

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),        # URDF + SRDF + kinematics + joint_limits
                                            # + moveit_controllers + planning pipelines
            move_group_configuration,
            {"use_sim_time": True},         # THE CRITICAL LINE — last dict wins
        ],
    )

    return LaunchDescription([move_group_node])

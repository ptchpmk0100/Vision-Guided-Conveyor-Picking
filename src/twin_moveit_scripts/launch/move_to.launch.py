from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(name="move_to", package="twin_moveit_scripts",
             executable="move_to", output="screen"),
    ])

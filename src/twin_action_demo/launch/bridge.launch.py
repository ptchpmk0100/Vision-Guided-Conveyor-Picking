from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Resolve the YAML path at launch time from the INSTALLED package share.
    # Same portable pattern as arm_bringup: no hardcoded absolute paths.
    bridge_config = PathJoinSubstitution([
        FindPackageShare('twin_action_demo'),
        'config',
        'panda_bridge.yaml',
    ])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    return LaunchDescription([bridge])
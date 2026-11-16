from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = LaunchConfiguration('model')
    extrinsic_file = LaunchConfiguration('extrinsic_file')
    min_confidence = LaunchConfiguration('min_confidence')
    output_topic = LaunchConfiguration('output_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=''),
        DeclareLaunchArgument(
            'extrinsic_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('twin_action_demo'),
                'config',
                'overhead_camera_extrinsic.yaml',
            ]),
        ),
        DeclareLaunchArgument('min_confidence', default_value='0.75'),
        DeclareLaunchArgument('output_topic', default_value='/parcel_cell/condition'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='twin_condition_cpp',
            executable='condition_node',
            output='screen',
            parameters=[{
                'model': model,
                'extrinsic_file': extrinsic_file,
                'min_confidence': min_confidence,
                'output_topic': output_topic,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node

# Use the FindPackageShare substitution to locate the package share directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Path to the URDF, resolved at launch time via the installed package.
    # We use a shell command substitution style: the create node takes -file.
    # urdf_path = (
    #     '/home/pmkhoi/ros_test_ws/install/twin_description/share/'
    #     'twin_description/urdf/one_joint_arm.urdf'
    # )
    
    # Resolve the URDF path at launch time from the installed package —
    # no hardcoded home directory, works on any machine.
    urdf_path = PathJoinSubstitution([
        FindPackageShare('twin_description'),
        'urdf',
        'one_joint_arm.urdf',
    ])

    # 1) SPAWN the arm into the already-running Gazebo world.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'empty',
            '-name', 'one_joint_arm',
            '-file', urdf_path,
            '-z', '0.0',
        ],
        output='screen',
    )

    # 2) BRIDGE — both directions in one node.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/empty/model/one_joint_arm/joint_state'
            '@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/one_joint_arm/joint/joint1/cmd_pos'
            '@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen',
    )

    # 3) ACTION SERVER — your node.
    server = Node(
        package='twin_action_demo',
        executable='move_joint_server',
        output='screen',
    )

    return LaunchDescription([spawn, bridge, server])
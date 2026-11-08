import os
from glob import glob

from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


GZ_CONFIG_PATHS = ':'.join(sorted(glob('/opt/ros/jazzy/opt/*_vendor/share/gz')))
GZ_LIB_PATHS = ':'.join(sorted(glob('/opt/ros/jazzy/opt/*_vendor/lib')))
GZ_TOOLS_BIN = '/opt/ros/jazzy/opt/gz_tools_vendor/bin'


def generate_launch_description():
    set_gz_version = SetEnvironmentVariable('GZ_VERSION', 'harmonic')
    set_gz_config = SetEnvironmentVariable('GZ_CONFIG_PATH', GZ_CONFIG_PATHS)
    append_ld = AppendEnvironmentVariable('LD_LIBRARY_PATH', GZ_LIB_PATHS)
    append_path = AppendEnvironmentVariable('PATH', GZ_TOOLS_BIN)

    world_path = PathJoinSubstitution([
        FindPackageShare('twin_action_demo'), 'worlds', 'sensors_empty.sdf',
    ])
    # HEADLESS: server-only + headless rendering (no GUI throttle) -- S21 recipe.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
            ])
        ),
        launch_arguments={'gz_args': [world_path, ' -r -s --headless-rendering']}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen',
    )

    urdf_path = PathJoinSubstitution([
        FindPackageShare('twin_description'), 'urdf', 'panda.urdf',
    ])
    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-world', 'empty', '-name', 'panda', '-file', urdf_path, '-z', '0.0'],
        output='screen',
    )
    arm_spawner = Node(package='controller_manager', executable='spawner',
                       arguments=['arm_controller'], output='screen')
    gripper_spawner = Node(package='controller_manager', executable='spawner',
                           arguments=['gripper_controller'], output='screen')
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[
            {'robot_description': ParameterValue(Command(['cat ', urdf_path]), value_type=str)},
            {'use_sim_time': True},
        ],
    )
    jsb_spawner = Node(package='controller_manager', executable='spawner',
                       arguments=['joint_state_broadcaster'], output='screen')
    jsb_after_spawn = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb_spawner]))
    arm_after_jsb = RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner]))
    gripper_after_jsb = RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[gripper_spawner]))

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('twin_action_demo'), 'launch', 'bridge.launch.py',
            ])
        )
    )

    # S22 additions under test: detector folded into bringup + cube spawned with world.
    object_detector = Node(
        package='twin_moveit_scripts', executable='object_detector', output='screen',
    )
    cube_path = PathJoinSubstitution([
        FindPackageShare('twin_action_demo'), 'models', 'grasp_box.sdf',
    ])
    spawn_cube = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-world', 'empty', '-name', 'b', '-file', cube_path,
                   '-x', '0.45', '-y', '0.0', '-z', '0.03'],
        output='screen',
    )
    cube_after_spawn = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[spawn_cube]))

    return LaunchDescription([
        set_gz_version, set_gz_config, append_ld, append_path,
        gz_sim, clock_bridge, robot_state_publisher, spawn,
        jsb_after_spawn, arm_after_jsb, gripper_after_jsb,
        cube_after_spawn, bridge, object_detector,
    ])

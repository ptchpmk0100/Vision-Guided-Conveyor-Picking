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


# --- Reproduce gz_env.sh's glob logic in Python, at launch time ---
# Same globs as ~/gz_env.sh: collect vendored gz config + lib dirs.
GZ_CONFIG_PATHS = ':'.join(
    sorted(glob('/opt/ros/jazzy/opt/*_vendor/share/gz'))
)
GZ_LIB_PATHS = ':'.join(
    sorted(glob('/opt/ros/jazzy/opt/*_vendor/lib'))
)
GZ_TOOLS_BIN = '/opt/ros/jazzy/opt/gz_tools_vendor/bin'


def generate_launch_description():
    # 1) gz environment (what gz_env.sh does, but baked in) ----------
    set_gz_version = SetEnvironmentVariable('GZ_VERSION', 'harmonic')
    set_gz_config = SetEnvironmentVariable('GZ_CONFIG_PATH', GZ_CONFIG_PATHS)
    # APPEND so we don't clobber any existing LD_LIBRARY_PATH / PATH.
    append_ld = AppendEnvironmentVariable('LD_LIBRARY_PATH', GZ_LIB_PATHS)
    append_path = AppendEnvironmentVariable('PATH', GZ_TOOLS_BIN)

    # 2) Start Gazebo by including ros_gz_sim's gz_sim.launch.py ------
    # S20 PERCEPTION: launch our sensors-enabled world, NOT bare empty.sdf.
    # Stock empty.sdf has no gz-sim-sensors-system, so camera <sensor> tags in
    # the spawned Panda never render -> /overhead_camera/* topics exist (the
    # bridge makes them) but publish nothing. sensors_empty.sdf is empty.sdf +
    # the sensors system plugin. Installed to this package's share/worlds.
    world_path = PathJoinSubstitution([
        FindPackageShare('twin_action_demo'),
        'worlds',
        'sensors_empty.sdf',
    ])
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            ])
        ),
        launch_arguments={'gz_args': [world_path, ' -r']}.items(),
    )

    # 2b) Clock bridge: gz owns sim-time, ROS consumes it. One-way gz -> ROS.
    #     Without this, /clock has 0 publishers and every use_sim_time node
    #     hangs waiting for a clock that never arrives.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # 3) Spawn the Panda (same as your manual `create` command) ------
    urdf_path = PathJoinSubstitution([
        FindPackageShare('twin_description'),
        'urdf',
        'panda.urdf',
    ])
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'empty',
            '-name', 'panda',
            '-file', urdf_path,
            '-z', '0.0',
        ],
        output='screen',
    )

    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
    )

    # gripper
    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen',
    )
    

    # 3b) robot_state_publisher: publishes URDF on /robot_description (latched)
    #     so gz_ros2_control's controller_manager can load the hardware.
    robot_description_content = ParameterValue(
        Command(['cat ', urdf_path]),
        value_type=str,
    )

    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': robot_description_content}],
    # )

    robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[
        {'robot_description': ParameterValue(
            Command(['cat ', urdf_path]), value_type=str)},
        {'use_sim_time': True},          # <-- add this dict
    ],
    )

    # 3c) Controller spawners, chained via event handlers so they only run
    #     AFTER the manager is up (spawn exits -> broadcaster -> position ctrl).
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    # jpc_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_position_controller'],
    #     output='screen',
    # )

    # After the robot finishes spawning, activate the broadcaster.
    jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[jsb_spawner])
    )

    # After the broadcaster is up, activate the position controller.
    # jpc_after_jsb = RegisterEventHandler(
    #     OnProcessExit(target_action=jsb_spawner, on_exit=[jpc_spawner])
    # )

    arm_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])
    )

    gripper_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[gripper_spawner])
    )

    # 3d) Finger state publisher: the 2 gripper joints (panda_finger_joint1/2)
    #     are NOT in <ros2_control>, so the broadcaster never reports them and
    #     move_group's planning-scene monitor blocks on 'missing finger joint'.
    #     This node owns ONLY those 2 joints (no overlap with the broadcaster's
    #     7 arm joints), publishing them at a fixed mid-range position.
    finger_state_publisher = Node(
        package='twin_action_demo',
        executable='finger_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 4) Include the bridge we already built + proved ----------------
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('twin_action_demo'),
                'launch',
                'bridge.launch.py',
            ])
        )
    )

    # S22: run the object detector as PART of bringup, so perception is always up
    # and repeated pick_and_place runs need no separate terminal. Run exactly as
    # S21 verified it via `ros2 run` (wall-clock stamps; move_to judges pose
    # freshness clock-independently, so no use_sim_time is needed here).
    object_detector = Node(
        package='twin_moveit_scripts',
        executable='object_detector',
        output='screen',
    )

    # S22: spawn the graspable cube as part of the world, at the start spot
    # (0.45, 0.0). -z 0.03 drops it so it settles with its center at 0.015. The
    # cube PERSISTS across pick_and_place runs, so the camera re-finds it wherever
    # the previous run dropped it -- no respawn needed. Installed to this package's
    # share/models by setup.py.
    cube_path = PathJoinSubstitution([
        FindPackageShare('twin_action_demo'),
        'models',
        'grasp_box.sdf',
    ])
    spawn_cube = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'empty',
            '-name', 'b',
            '-file', cube_path,
            '-x', '0.45', '-y', '0.0', '-z', '0.03',
        ],
        output='screen',
    )
    # Spawn the cube only AFTER the Panda create exits -- by then the world is up.
    cube_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[spawn_cube])
    )

    return LaunchDescription([
        set_gz_version,
        set_gz_config,
        append_ld,
        append_path,
        gz_sim,
        clock_bridge,          # <-- add, early so /clock is live before the manager
        robot_state_publisher,
        spawn,
        jsb_after_spawn,
        # jpc_after_jsb,
        arm_after_jsb,
        # finger_state_publisher,
        gripper_after_jsb,
        cube_after_spawn,      # S22: cube spawns once the Panda is in the world
        bridge,
        object_detector,       # S22: perception up as part of bringup
    ])
"""A1 sorting cell bringup: world, bins, gantry, robot, camera, cube and detector.

Separate from panda_bringup.launch.py so the verified empty-world bringup stays as it is.

  ros2 launch twin_action_demo sorting_cell_bringup.launch.py                 # Gazebo GUI
  ros2 launch twin_action_demo sorting_cell_bringup.launch.py headless:=true  # no GUI
"""
from glob import glob

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

GZ_CONFIG_PATHS = ':'.join(sorted(glob('/opt/ros/jazzy/opt/*_vendor/share/gz')))
GZ_LIB_PATHS = ':'.join(sorted(glob('/opt/ros/jazzy/opt/*_vendor/lib')))
GZ_TOOLS_BIN = '/opt/ros/jazzy/opt/gz_tools_vendor/bin'

# Open pick area (PLACE_SLOT 1). The empty-world start (0.45, 0) touches the bin walls.
CUBE_START_X = '0.50'
CUBE_START_Y = '0.0'


def generate_launch_description():
    share = FindPackageShare('twin_action_demo')
    world_path = PathJoinSubstitution([share, 'worlds', 'sorting_cell.sdf'])
    cube_path = PathJoinSubstitution([share, 'models', 'grasp_box.sdf'])
    urdf_path = PathJoinSubstitution(
        [FindPackageShare('twin_description'), 'urdf', 'panda.urdf'])

    # Two explicit includes guarded by conditions. A PythonExpression that built the
    # flag string inline silently evaluated to '' -- the GUI started even with
    # headless:=true, which throttled the sim and left orphan gz processes behind.
    # Each path now carries a hardcoded, known-good gz_args string.
    gz_launch = PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']))
    gz_sim_gui = IncludeLaunchDescription(
        gz_launch,
        launch_arguments={'gz_args': [world_path, ' -r']}.items(),
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )
    gz_sim_headless = IncludeLaunchDescription(
        gz_launch,
        launch_arguments={'gz_args': [world_path, ' -r -s --headless-rendering']}.items(),
        condition=IfCondition(LaunchConfiguration('headless')),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': ParameterValue(
                Command(['cat ', urdf_path]), value_type=str)},
            {'use_sim_time': True},
        ],
    )
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'empty', '-name', 'panda', '-file', urdf_path, '-z', '0.0'],
        output='screen',
    )
    spawn_cube = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'empty', '-name', 'b', '-file', cube_path,
                   '-x', CUBE_START_X, '-y', CUBE_START_Y, '-z', '0.03'],
        output='screen',
    )

    def spawner(controller):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller],
            output='screen',
        )

    jsb_spawner = spawner('joint_state_broadcaster')

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([share, 'launch', 'bridge.launch.py'])))
    object_detector = Node(
        package='twin_moveit_scripts',
        executable='object_detector',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='true = no Gazebo GUI (full real-time speed)'),
        SetEnvironmentVariable('GZ_VERSION', 'harmonic'),
        SetEnvironmentVariable('GZ_CONFIG_PATH', GZ_CONFIG_PATHS),
        AppendEnvironmentVariable('LD_LIBRARY_PATH', GZ_LIB_PATHS),
        AppendEnvironmentVariable('PATH', GZ_TOOLS_BIN),
        gz_sim_gui,
        gz_sim_headless,
        clock_bridge,
        robot_state_publisher,
        spawn_robot,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot, on_exit=[jsb_spawner, spawn_cube])),
        RegisterEventHandler(OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[spawner('arm_controller'), spawner('gripper_controller')])),
        bridge,
        object_detector,
    ])

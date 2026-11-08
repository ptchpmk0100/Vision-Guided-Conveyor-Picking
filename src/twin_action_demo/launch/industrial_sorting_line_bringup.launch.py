"""V2 sorting-line bringup: raised cell + 3 colour boxes on the pick deck.

Additive -- the A1/A2 launches and `sort_task.py` are untouched. Boxes spawn above the
pick deck (top z=0.295) and settle with their centres at z=0.320. Reach to every box and
tray pose was ground-truth verified GO at this height (V2_DECISIONS.md D5).

  ros2 launch twin_action_demo industrial_sorting_line_bringup.launch.py                 # Stage A geometry
  ros2 launch twin_action_demo industrial_sorting_line_bringup.launch.py headless:=true
  ros2 launch twin_action_demo industrial_sorting_line_bringup.launch.py detector:=true  # Stage B+ (needs object_detector_sorting_line)
  ros2 launch twin_action_demo industrial_sorting_line_bringup.launch.py detector:=true \
    extrinsic_file:=$(ros2 pkg prefix twin_action_demo)/share/twin_action_demo/config/overhead_camera_extrinsic.yaml
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
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

GZ_CONFIG_PATHS = ':'.join(sorted(glob('/opt/ros/jazzy/opt/*_vendor/share/gz')))
GZ_LIB_PATHS = ':'.join(sorted(glob('/opt/ros/jazzy/opt/*_vendor/lib')))
GZ_TOOLS_BIN = '/opt/ros/jazzy/opt/gz_tools_vendor/bin'

# entity name -> (model stem, x, y). Belt-height staging poses (reach-verified GO).
OBJECTS = [
    ("obj_red_box_1",    "obj_red_box",    0.560,  0.110),
    ("obj_green_box_1",  "obj_green_box",  0.560,  0.000),
    ("obj_yellow_box_1", "obj_yellow_box", 0.560, -0.110),
]
SPAWN_Z = '0.345'   # 25 mm above the deck top (0.295); settles to centre z=0.320


def generate_launch_description():
    share = FindPackageShare('twin_action_demo')
    world_path = PathJoinSubstitution([share, 'worlds', 'industrial_sorting_line.sdf'])
    urdf_path = PathJoinSubstitution(
        [FindPackageShare('twin_description'), 'urdf', 'panda.urdf'])

    gz_launch = PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']))
    gz_sim_gui = IncludeLaunchDescription(
        gz_launch, launch_arguments={'gz_args': [world_path, ' -r']}.items(),
        condition=UnlessCondition(LaunchConfiguration('headless')))
    gz_sim_headless = IncludeLaunchDescription(
        gz_launch,
        launch_arguments={'gz_args': [world_path, ' -r -s --headless-rendering']}.items(),
        condition=IfCondition(LaunchConfiguration('headless')))

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen')
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[
            {'robot_description': ParameterValue(Command(['cat ', urdf_path]), value_type=str)},
            {'use_sim_time': True},
        ])
    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-world', 'empty', '-name', 'panda', '-file', urdf_path, '-z', '0.0'],
        output='screen')

    object_spawns = [
        Node(package='ros_gz_sim', executable='create',
             arguments=['-world', 'empty', '-name', name,
                        '-file', PathJoinSubstitution([share, 'models', f'{stem}.sdf']),
                        '-x', str(x), '-y', str(y), '-z', SPAWN_Z],
             output='screen')
        for name, stem, x, y in OBJECTS
    ]

    def spawner(controller):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[controller], output='screen')

    jsb_spawner = spawner('joint_state_broadcaster')

    bridge = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([share, 'launch', 'bridge.launch.py'])))
    # Stage B+: the raised-scene detector. Off by default so Stage A is geometry-only.
    detector = Node(package='twin_moveit_scripts', executable='object_detector_sorting_line',
                    parameters=[{'extrinsic_file': ParameterValue(
                        LaunchConfiguration('extrinsic_file'), value_type=str)}],
                    output='screen', condition=IfCondition(LaunchConfiguration('detector')))

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='true = no Gazebo GUI (full real-time speed)'),
        DeclareLaunchArgument('detector', default_value='false',
                              description='true = start object_detector_sorting_line (Stage B+)'),
        DeclareLaunchArgument(
            'extrinsic_file', default_value='',
            description=('optional world<-camera optical YAML; empty preserves the '
                         'verified hardcoded transform')),
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
            target_action=spawn_robot, on_exit=[jsb_spawner] + object_spawns)),
        RegisterEventHandler(OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[spawner('arm_controller'), spawner('gripper_controller')])),
        bridge,
        detector,
    ])

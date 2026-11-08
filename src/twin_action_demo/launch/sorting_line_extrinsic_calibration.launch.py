"""B1 extrinsic-calibration scene: camera, Panda, neutral fixture, and 12 flat tiles.

No detector, sorter, or calibration command is started automatically.  After a clean
mirror build, the manager runs:

  ros2 launch twin_action_demo sorting_line_extrinsic_calibration.launch.py headless:=true

Then, in another identically sourced shell:

  python3 /home/pmkhoi/ros_test_ws_mirror/tools/collect_extrinsic_calibration.py \
    --run-id b1_run_01 --target-half-height 0.002

The camera comes from the unchanged twin_description Panda URDF and is therefore at the
same world pose with the same RGBD parameters as production.
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

# name, calibration model stem, x, y, spawn z.  Four-millimetre tiles are spawned
# directly at rest: base centres z=0.297 and raised centres z=0.347 (no drop/bounce).
CALIBRATION_OBJECTS = [
    ("cal_01_red_tile", "obj_red_tile", 0.3700, -0.150, 0.297),
    ("cal_02_green_tile", "obj_green_tile", 0.3700, 0.000, 0.297),
    ("cal_03_yellow_tile", "obj_yellow_tile", 0.3700, 0.150, 0.297),
    ("cal_04_red_tile", "obj_red_tile", 0.4650, -0.150, 0.297),
    ("cal_05_green_tile", "obj_green_tile", 0.4650, 0.000, 0.297),
    ("cal_06_yellow_tile", "obj_yellow_tile", 0.4650, 0.150, 0.297),
    ("cal_07_red_tile", "obj_red_tile", 0.5600, -0.150, 0.297),
    ("cal_08_green_tile", "obj_green_tile", 0.5600, 0.000, 0.297),
    ("cal_09_yellow_tile", "obj_yellow_tile", 0.5600, 0.150, 0.297),
    ("cal_10_red_tile", "obj_red_tile", 0.4175, -0.075, 0.347),
    ("cal_11_green_tile", "obj_green_tile", 0.5125, -0.075, 0.347),
    ("cal_12_yellow_tile", "obj_yellow_tile", 0.4175, 0.075, 0.347),
]


def generate_launch_description():
    share = FindPackageShare('twin_action_demo')
    world_path = PathJoinSubstitution([
        share, 'worlds', 'industrial_sorting_line_calibration.sdf',
    ])
    urdf_path = PathJoinSubstitution([
        FindPackageShare('twin_description'), 'urdf', 'panda.urdf',
    ])

    gz_launch = PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
    ]))
    gz_sim_gui = IncludeLaunchDescription(
        gz_launch,
        launch_arguments={'gz_args': [world_path, ' -r']}.items(),
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )
    gz_sim_headless = IncludeLaunchDescription(
        gz_launch,
        launch_arguments={
            'gz_args': [world_path, ' -r -s --headless-rendering'],
        }.items(),
        condition=IfCondition(LaunchConfiguration('headless')),
    )

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
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
        package='ros_gz_sim', executable='create',
        arguments=[
            '-world', 'empty', '-name', 'panda', '-file', urdf_path,
            '-z', '0.0',
        ],
        output='screen',
    )

    object_spawns = [
        Node(
            package='ros_gz_sim', executable='create',
            arguments=[
                '-world', 'empty', '-name', name,
                '-file', PathJoinSubstitution([
                    share, 'models', f'{model_stem}.sdf',
                ]),
                '-x', str(x), '-y', str(y), '-z', str(z),
            ],
            output='screen',
        )
        for name, model_stem, x, y, z in CALIBRATION_OBJECTS
    ]

    def spawner(controller):
        return Node(
            package='controller_manager', executable='spawner',
            arguments=[controller], output='screen',
        )

    jsb_spawner = spawner('joint_state_broadcaster')
    bridge = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([share, 'launch', 'bridge.launch.py'])
    ))

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true = server-only headless rendering',
        ),
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
            target_action=spawn_robot,
            on_exit=[jsb_spawner] + object_spawns,
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[spawner('arm_controller'), spawner('gripper_controller')],
        )),
        bridge,
    ])

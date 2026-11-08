"""A2 bringup: industrial cell + a MIXED OBJECT SET + the multi-object HSV detector.

Additive by design -- `industrial_cell_bringup.launch.py` (A1, ground-truth verified)
is untouched and still runnable. This launch differs in exactly three ways:
  * no single blue cube,
  * three mixed-colour/shape objects on the verified staging row,
  * `object_detector_multi` instead of the single-blue `object_detector`.

Staging row (reach-verified 2/2 by tools/reach_probe.py): x = 0.56, y in [-0.10, +0.10].
Objects sit 0.10 m apart: the hand opens to +/-0.04 m from the grasp centre, so a
neighbour's near edge at 0.085 m stays clear of the finger path (0.05 m spacing would
NOT -- the edge lands at 0.035 m, inside the fingers).

Bins (reach-verified 2/2): bin_a (0.45, +0.18), bin_b (0.45, -0.18), bin_c (0.37, 0).
A back row at (0.37, +/-0.20) was tried and is GRASP-Z BLOCKED -- see V2_RESUME.md.

  ros2 launch twin_action_demo industrial_cell_a2_bringup.launch.py
  ros2 launch twin_action_demo industrial_cell_a2_bringup.launch.py headless:=true
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

# entity name -> (model file stem, x, y). The ENTITY NAME IS THE LABEL:
# tools/check_detections.py parses obj_<colour>_<shape>_<n> to score the detector
# against ground truth, so no hand-annotation is needed.
OBJECTS = [
    ("obj_red_cube_1",       "obj_red_cube",       0.56,  0.10),
    ("obj_green_cylinder_1", "obj_green_cylinder", 0.56,  0.00),
    ("obj_yellow_cube_1",    "obj_yellow_cube",    0.56, -0.10),
]
SPAWN_Z = '0.03'   # drops and settles with its centre at the 0.015 invariant


def generate_launch_description():
    share = FindPackageShare('twin_action_demo')
    world_path = PathJoinSubstitution([share, 'worlds', 'industrial_cell.sdf'])
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
    detector = Node(package='twin_moveit_scripts', executable='object_detector_multi',
                    output='screen')

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
            target_action=spawn_robot, on_exit=[jsb_spawner] + object_spawns)),
        RegisterEventHandler(OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[spawner('arm_controller'), spawner('gripper_controller')])),
        bridge,
        detector,
    ])

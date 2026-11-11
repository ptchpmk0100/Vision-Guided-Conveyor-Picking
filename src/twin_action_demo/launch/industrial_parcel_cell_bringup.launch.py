"""Additive parcel cell; unchanged Panda geometry with workspace-local controllers."""
from glob import glob
from pathlib import Path
import tempfile
import hashlib
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    share = Path(get_package_share_directory('twin_action_demo'))
    description = Path(get_package_share_directory('twin_description'))
    from twin_moveit_scripts.parcel_layout import batch_layout
    pitch=LaunchConfiguration('parcel_pitch').perform(context)
    batch=batch_layout(int(LaunchConfiguration('parcel_count').perform(context)),
                       float(pitch) if pitch else None)
    text = (description / 'urdf/panda.urdf').read_text()
    # Compatibility fix confined to this launch: the stored URDF embeds the primary
    # workspace controller YAML. Change only that resource path, never robot structure.
    parsed = ET.fromstring(text)
    parameters = parsed.find(".//plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']/parameters")
    if parameters is None:
        raise RuntimeError('Panda control plugin parameters were not found')
    text = text.replace(parameters.text, str(description / 'config/panda_controllers.yaml'))
    if batch.extended:
        parsed=ET.fromstring(text)
        parsed.find(".//sensor[@name='overhead_rgbd']/camera/horizontal_fov").text=str(batch.camera_hfov)
        text=ET.tostring(parsed,encoding='unicode')
    if LaunchConfiguration('grasp_sensors').perform(context).lower() == 'true' or batch.extended:
        from twin_moveit_scripts.parcel_sensors import add_pad_sensors
        text = add_pad_sensors(text)
    temp = tempfile.NamedTemporaryFile(mode='w', prefix='panda-parcel-', suffix='.urdf', delete=False)
    temp.write(text)
    temp.close()
    source = PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim'))
                                               / 'launch/gz_sim.launch.py'))
    continuous = LaunchConfiguration('continuous').perform(context).lower() == 'true'
    indexed = LaunchConfiguration('indexed').perform(context).lower() == 'true'
    if continuous and indexed:
        raise RuntimeError('Choose continuous:=true or indexed:=true, not both')
    if batch.count!=3 and not continuous:
        raise RuntimeError('7/8/10-parcel profiles require continuous:=true')
    world = str(share / 'worlds' / ('industrial_parcel_cell_continuous.sdf' if continuous
                                   else 'industrial_parcel_cell.sdf'))
    world_temp=None
    reject_bin = LaunchConfiguration('reject_bin').perform(context).lower() == 'true'
    if reject_bin and not batch.extended:
        raise RuntimeError('reject_bin:=true needs parcel_count 7, 8 or 10')
    if batch.extended:
        from twin_moveit_scripts.parcel_batch_world import write_batch_world
        handle=tempfile.NamedTemporaryFile(prefix='parcel-ten-',suffix='.sdf',delete=False)
        handle.close();world_temp=Path(handle.name)
        write_batch_world(world,world_temp,batch,reject_bin=reject_bin);world=str(world_temp)
    gui = IncludeLaunchDescription(source, launch_arguments={'gz_args': world + ' -r'}.items(),
                                   condition=UnlessCondition(LaunchConfiguration('headless')))
    headless = IncludeLaunchDescription(source,
        launch_arguments={'gz_args': world + ' -r -s --headless-rendering'}.items(),
        condition=IfCondition(LaunchConfiguration('headless')))
    robot = Node(package='ros_gz_sim', executable='create',
                 arguments=['-world', 'empty', '-name', 'panda', '-file', temp.name], output='screen')
    def controller(name):
        return Node(package='controller_manager', executable='spawner', arguments=[name], output='screen')
    jsb = controller('joint_state_broadcaster')
    from twin_moveit_scripts.parcel_layout import PICK_X, QUEUE_Y, STAGING_Y, CONTINUOUS_QUEUE_Y
    ys = batch.queue_y if continuous else (QUEUE_Y if indexed else STAGING_Y)
    # Optional appearance variants (default OFF -> every parcel is the original parcel_box.sdf).
    variant_files, variant_yaw, variant_dir = {}, {}, None
    variant_seed = LaunchConfiguration('variant_seed').perform(context)
    if variant_seed:
        import json, random
        from twin_moveit_scripts.parcel_variants import parse_damaged, write_variants
        damaged = parse_damaged(LaunchConfiguration('variant_damaged').perform(context), batch.count)
        variant_dir = tempfile.mkdtemp(prefix='parcel-variants-')
        variant_files, manifest = write_variants(variant_dir, share / 'models/parcel_box.sdf',
                                                 batch.count, damaged, variant_seed)
        yaw_limit = float(LaunchConfiguration('variant_yaw_deg').perform(context))
        rng = random.Random(f'{variant_seed}:yaw')
        for info in manifest:
            info['yaw_deg'] = round(rng.uniform(-yaw_limit, yaw_limit), 2)
            variant_yaw[info['name']] = info['yaw_deg']
        path = LaunchConfiguration('variant_manifest').perform(context)
        if path:
            Path(path).write_text(json.dumps({'seed': variant_seed, 'count': batch.count,
                                              'queue_y': list(ys), 'parcels': manifest}, indent=1))
    parcels = [Node(package='ros_gz_sim', executable='create',
                    arguments=['-world', 'empty', '-name', f'parcel_{i}', '-file',
                               variant_files.get(f'parcel_{i}', str(share / 'models/parcel_box.sdf')),
                               '-x', str(PICK_X), '-y', str(y), '-z', '.345',
                               *(['-Y', str(variant_yaw[f'parcel_{i}'] * 3.141592653589793 / 180)]
                                 if variant_yaw.get(f'parcel_{i}') else [])], output='screen')
               for i, y in enumerate(ys, 1)]
    bridges = [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/bridge.launch.py'))),
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/parcel_cell/belt_speed@std_msgs/msg/Float64]gz.msgs.Double',
            '/parcel_cell/overview@sensor_msgs/msg/Image[gz.msgs.Image',
            '/parcel_cell/side@sensor_msgs/msg/Image[gz.msgs.Image'], output='screen')]
    def cleanup(_):
        Path(temp.name).unlink(missing_ok=True)
        if world_temp is not None:world_temp.unlink(missing_ok=True)
        if variant_dir is not None:
            import shutil
            shutil.rmtree(variant_dir, ignore_errors=True)
        return []
    return [gui, headless,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': text, 'use_sim_time': True}]),
        robot, *bridges,
        Node(package='twin_moveit_scripts', executable='parcel_feed_guard', output='screen',
             parameters=[{'max_speed': .08 if continuous else .04}]),
        RegisterEventHandler(OnProcessExit(target_action=robot, on_exit=[jsb, *parcels])),
        RegisterEventHandler(OnProcessExit(target_action=jsb,
                                          on_exit=[controller('arm_controller'), controller('gripper_controller')])),
        Node(package='twin_moveit_scripts', executable='object_detector_parcel_cell',
             parameters=[{'use_sim_time': True,
                          'continuous': continuous,
                          'parcel_count':batch.count,'parcel_pitch':batch.pitch,
                          'batch_scene_hash':hashlib.sha256(Path(world).read_bytes()).hexdigest(),
                          'extrinsic_file': str(share / 'config/overhead_camera_extrinsic.yaml')}],
             condition=IfCondition(LaunchConfiguration('detector')), output='screen'),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
            function=cleanup)]))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('detector', default_value='true'),
        DeclareLaunchArgument('indexed', default_value='false', description='true = parcels queued upstream'),
        DeclareLaunchArgument('continuous', default_value='false', description='true = spaced parcels on extended belt'),
        DeclareLaunchArgument('grasp_sensors', default_value='false'),
        DeclareLaunchArgument('reject_bin', default_value='false', description='true = add the reject bin (condition routing)'),
        DeclareLaunchArgument('variant_seed', default_value='', description='non-empty = intact/damaged appearance variants'),
        DeclareLaunchArgument('variant_damaged', default_value='', description='comma list of 1-based damaged parcel indices'),
        DeclareLaunchArgument('variant_manifest', default_value='', description='write the ground-truth variant manifest (json) here'),
        DeclareLaunchArgument('variant_yaw_deg', default_value='0', description='spawn yaw jitter half-range (deg), data collection only'),
        DeclareLaunchArgument('parcel_count', default_value='3'),
        DeclareLaunchArgument('parcel_pitch', default_value='', description='Continuous centre-to-centre spacing (m)'),
        SetEnvironmentVariable('GZ_VERSION', 'harmonic'),
        SetEnvironmentVariable('GZ_CONFIG_PATH', ':'.join(sorted(glob('/opt/ros/jazzy/opt/*_vendor/share/gz')))),
        AppendEnvironmentVariable('LD_LIBRARY_PATH', ':'.join(sorted(glob('/opt/ros/jazzy/opt/*_vendor/lib')))),
        AppendEnvironmentVariable('PATH', '/opt/ros/jazzy/opt/gz_tools_vendor/bin'),
        OpaqueFunction(function=_launch)])

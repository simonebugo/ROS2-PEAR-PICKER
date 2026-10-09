import os
import runpy
import secrets
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction, SetLaunchConfiguration, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def prepare_scenario(context):
    share = get_package_share_directory('rbt08pkg')
    helper = os.path.join(get_package_prefix('rbt08pkg'), 'lib', 'rbt08pkg', 'orchard_randomization.py')
    previous_path = sys.path[:]
    try:
        sys.path.insert(0, os.path.dirname(helper))
        api = runpy.run_path(helper)
    finally:
        sys.path[:] = previous_path
    mode = LaunchConfiguration('grasp_orientation').perform(context)
    if LaunchConfiguration('randomize').perform(context) != 'true':
        return [SetLaunchConfiguration('orchard_pear_yaw_world', '0.0'),
                SetLaunchConfiguration('orchard_pear_pitch_world', '0.0')]
    if LaunchConfiguration('pose_source').perform(context) != 'vision':
        raise RuntimeError('randomize:=true requires pose_source:=vision: the target must come from the camera.')
    raw_seed = LaunchConfiguration('seed').perform(context)
    seed = secrets.randbits(32) if raw_seed == 'auto' else int(raw_seed)
    scenario = api['generate_scenario'](share, LaunchConfiguration('randomization_config').perform(context), seed, mode)
    directory = tempfile.mkdtemp(prefix=f'pear_orchard_{seed}_')
    world, manifest = api['write_scenario'](share, scenario, directory)
    return [
        SetLaunchConfiguration('orchard_world_file', world),
        SetLaunchConfiguration('orchard_scene_config', manifest),
        SetLaunchConfiguration('orchard_pear_yaw_world', str(scenario['pear_pose_world'][5])),
        SetLaunchConfiguration('orchard_pear_pitch_world', str(scenario['pear_pose_world'][4])),
        LogInfo(msg=f'ORCHARD seed={seed}, attempt={scenario["attempt"]}, pear={scenario["pear_pose_world"][:3]}'),
        LogInfo(msg=f'ORCHARD scenario={manifest} (position from vision; orientation mode={mode})'),
        LogInfo(msg='ORCHARD grasp offset policy=robot_position (projection in pear XY)'),
    ]


def generate_launch_description():
    moveit_share = get_package_share_directory('rbt10pkg_mit')
    robot_share = get_package_share_directory('rbt08pkg')
    vision = PythonExpression(["'true' if '", LaunchConfiguration('pose_source'), "' == 'vision' else 'false'"])

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_share,
                'launch',
                'gazebo_moveit_test.launch.py',
            )
        ),
        launch_arguments={
            'enable_physical_grasp': 'true',
            'enable_camera': vision,
            'render_engine': LaunchConfiguration('render_engine'),
            'world_file': LaunchConfiguration('orchard_world_file'),
            'scene_config': LaunchConfiguration('orchard_scene_config'),
        }.items(),
    )

    pear_pose_source = Node(
        package='rbt08pkg',
        executable='pear_pose_source.py',
        name='pear_pose_source',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(vision),
    )

    pick_place_manager = Node(
        package='rbt08pkg',
        executable='pick_place_manager.py',
        name='pick_place_manager',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'require_physical_grasp': True,
            'require_vision_status': ParameterValue(vision, value_type=bool),
            'max_pose_age_s': ParameterValue(LaunchConfiguration('max_pose_age_s'), value_type=float),
            'grasp_orientation': LaunchConfiguration('grasp_orientation'),
            # Leave a small clearance during DetachableJoint reattachment.
            # Gazebo / DART cannot reattach parent and child while colliding.
            'gripper_closed_position': 0.005,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('pose_source', default_value='static', choices=['static', 'vision']),
        DeclareLaunchArgument('max_pose_age_s', default_value='5.0'),
        DeclareLaunchArgument('render_engine', default_value='ogre', choices=['ogre', 'ogre2']),
        DeclareLaunchArgument('randomize', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('grasp_orientation', default_value='branch', choices=['branch', 'radial'],
                              description='Robot-side approach in oriented pear XY, or horizontal radial approach.'),
        DeclareLaunchArgument('seed', default_value='42', description='Integer seed, or auto for a new recorded seed.'),
        DeclareLaunchArgument('randomization_config', default_value=os.path.join(robot_share, 'config', 'orchard_randomization.json')),
        SetLaunchConfiguration('orchard_world_file', os.path.join(robot_share, 'worlds', 'orchard.world.sdf')),
        SetLaunchConfiguration('orchard_scene_config', ''),
        OpaqueFunction(function=prepare_scenario),
        simulation,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(robot_share, 'launch', 'vision_test.launch.py')),
            launch_arguments={
                'start_camera': 'false', 'rviz': 'false',
                'publish_harvest_pose': 'true',
                'pear_yaw_world': LaunchConfiguration('orchard_pear_yaw_world'),
                'pear_pitch_world': LaunchConfiguration('orchard_pear_pitch_world'),
            }.items(),
            condition=IfCondition(vision),
        ),
        # Start the application nodes after controllers, MoveIt and the
        # collision scene have had time to initialize on the VM.
        TimerAction(
            period=25.0,
            actions=[pear_pose_source, pick_place_manager],
        ),
    ])

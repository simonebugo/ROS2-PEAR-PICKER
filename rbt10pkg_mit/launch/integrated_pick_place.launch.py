import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
            # Leave a small clearance during DetachableJoint reattachment.
            # Gazebo / DART cannot reattach parent and child while colliding.
            'gripper_closed_position': 0.005,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('pose_source', default_value='static', choices=['static', 'vision']),
        DeclareLaunchArgument('max_pose_age_s', default_value='5.0'),
        DeclareLaunchArgument('render_engine', default_value='ogre', choices=['ogre', 'ogre2']),
        simulation,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(robot_share, 'launch', 'vision_test.launch.py')),
            launch_arguments={
                'start_camera': 'false', 'rviz': 'false',
                'publish_harvest_pose': 'true',
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

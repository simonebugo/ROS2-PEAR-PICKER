import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    rbt08_share = get_package_share_directory('rbt08pkg')
    enable_physical_grasp = LaunchConfiguration('enable_physical_grasp')
    moveit_config = (
        MoveItConfigsBuilder('rbt10', package_name='rbt10pkg_mit')
        .to_moveit_configs()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rbt08_share, 'launch', 'gazebo_robot.launch.py')
        ),
        launch_arguments={
            'enable_ros2_control': 'true',
            'enable_physical_grasp': enable_physical_grasp,
            'enable_camera': LaunchConfiguration('enable_camera'),
            'render_engine': LaunchConfiguration('render_engine'),
        }.items(),
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
        ],
    )

    planning_scene = Node(
        package='rbt08pkg',
        executable='spawn_planning_scene.py',
        name='planning_scene_spawner',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_camera', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('render_engine', default_value='ogre', choices=['ogre', 'ogre2']),
        DeclareLaunchArgument(
            'enable_physical_grasp',
            default_value='false',
            choices=['true', 'false'],
            description='Enable the Gazebo grasp lifecycle.',
        ),
        gazebo,
        # Give Gazebo time to create its controller manager before MoveIt
        # checks the FollowJointTrajectory action servers.
        TimerAction(period=15.0, actions=[move_group]),
        TimerAction(period=20.0, actions=[planning_scene]),
    ])

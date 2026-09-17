import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('rbt08pkg')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('check_camera', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'render_engine', default_value='ogre2', choices=['ogre', 'ogre2'],
            description='GUI renderer; the RGB-D sensor always uses ogre2.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_share, 'launch', 'gazebo_robot.launch.py')
            ),
            launch_arguments={
                'enable_camera': 'true',
                'enable_ros2_control': 'true',
                'enable_physical_grasp': 'false',
                'render_engine': LaunchConfiguration('render_engine'),
            }.items(),
        ),
        Node(
            package='rbt08pkg', executable='camera_pipeline_check.py',
            name='camera_pipeline_check', output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('check_camera')),
        ),
        Node(
            package='rviz2', executable='rviz2', name='camera_rviz',
            arguments=['-d', os.path.join(package_share, 'config', 'camera.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen', condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])

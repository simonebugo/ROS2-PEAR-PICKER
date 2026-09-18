import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('rbt08pkg')
    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rgb_topic', default_value='/orchard/camera/rgb/image_raw'),
        DeclareLaunchArgument('detector_config', default_value=os.path.join(share, 'config', 'pear_segmentation.yaml')),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('render_engine', default_value='ogre2', choices=['ogre', 'ogre2']),
        Node(
            package='rbt08pkg', executable='pear_detector.py', name='pear_detector',
            prefix='/usr/bin/python3', output='screen',
            parameters=[LaunchConfiguration('detector_config'), {
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'rgb_topic': LaunchConfiguration('rgb_topic'),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'camera_test.launch.py')),
            launch_arguments={
                'rviz': LaunchConfiguration('rviz'),
                'render_engine': LaunchConfiguration('render_engine'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_camera')),
        ),
    ])

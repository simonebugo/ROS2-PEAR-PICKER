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
        DeclareLaunchArgument('estimate_3d', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('transform_world', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('estimate_center', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('fruit_radius_m', default_value='0.075'),
        DeclareLaunchArgument('pear_yaw_world', default_value='0.0'),
        DeclareLaunchArgument('pear_pitch_world', default_value='0.0'),
        DeclareLaunchArgument('publish_harvest_pose', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('depth_config', default_value=os.path.join(share, 'config', 'pear_depth.yaml')),
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
        Node(
            package='rbt08pkg', executable='pear_surface_estimator.py', name='pear_surface_estimator',
            prefix='/usr/bin/python3', output='screen',
            parameters=[LaunchConfiguration('depth_config'), {
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
            condition=IfCondition(LaunchConfiguration('estimate_3d')),
        ),
        Node(
            package='rbt08pkg', executable='pear_world_transformer.py', name='pear_world_transformer',
            prefix='/usr/bin/python3', output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'estimate_center': ParameterValue(LaunchConfiguration('estimate_center'), value_type=bool),
                'fruit_radius_m': ParameterValue(LaunchConfiguration('fruit_radius_m'), value_type=float),
                'pear_yaw_world': ParameterValue(LaunchConfiguration('pear_yaw_world'), value_type=float),
                'pear_pitch_world': ParameterValue(LaunchConfiguration('pear_pitch_world'), value_type=float),
                'publish_harvest_pose': ParameterValue(LaunchConfiguration('publish_harvest_pose'), value_type=bool),
            }],
            condition=IfCondition(LaunchConfiguration('transform_world')),
        ),
    ])

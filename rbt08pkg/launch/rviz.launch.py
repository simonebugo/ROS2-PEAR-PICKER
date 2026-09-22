"""Inspect the robot and move its joints in RViz without a simulator."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('rbt08pkg')
    gui = LaunchConfiguration('gui')
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('model', default_value='rbt10.urdf'),
        DeclareLaunchArgument('rvizconfig', default_value=os.path.join(share, 'config', 'config.rviz')),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[clock, {'robot_description': ParameterValue(Command([
                 'xacro ', PathJoinSubstitution([share, 'urdf', LaunchConfiguration('model')])
             ]), value_type=str)}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             parameters=[clock], condition=IfCondition(gui)),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[clock], condition=UnlessCondition(gui)),
        Node(package='rviz2', executable='rviz2', parameters=[clock],
             arguments=['-d', LaunchConfiguration('rvizconfig')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    moveit_share = get_package_share_directory('rbt10pkg_mit')

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
        }.items(),
    )

    pear_pose_source = Node(
        package='rbt08pkg',
        executable='pear_pose_source.py',
        name='pear_pose_source',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    pick_place_manager = Node(
        package='rbt08pkg',
        executable='pick_place_manager.py',
        name='pick_place_manager',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'require_physical_grasp': True,
            # Leave a small clearance during DetachableJoint reattachment.
            # Gazebo / DART cannot reattach parent and child while colliding.
            'gripper_closed_position': 0.005,
        }],
    )

    return LaunchDescription([
        simulation,
        # Start the application nodes after controllers, MoveIt and the
        # collision scene have had time to initialize on the VM.
        TimerAction(
            period=25.0,
            actions=[pear_pose_source, pick_place_manager],
        ),
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('rbt08pkg')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    enable_ros2_control = LaunchConfiguration('enable_ros2_control')
    enable_physical_grasp = LaunchConfiguration('enable_physical_grasp')
    enable_camera = LaunchConfiguration('enable_camera')
    render_engine = LaunchConfiguration('render_engine')

    urdf_path = PathJoinSubstitution([
        package_share,
        'urdf',
        model,
    ])
    world_path = PathJoinSubstitution([
        package_share,
        'worlds',
        world,
    ])
    clock_bridge_config = os.path.join(
        package_share,
        'config',
        'gz_bridge_clock.yaml',
    )
    grasp_bridge_config = os.path.join(
        package_share,
        'config',
        'gz_bridge.yaml',
    )
    camera_bridge_config = os.path.join(
        package_share, 'config', 'gz_bridge_camera.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='orchard.world.sdf',
            description='Gazebo world file from rbt08pkg/worlds.',
        ),
        DeclareLaunchArgument(
            'model',
            default_value='rbt10.urdf',
            description='Robot xacro/URDF file from rbt08pkg/urdf.',
        ),
        DeclareLaunchArgument(
            'enable_ros2_control',
            default_value='false',
            choices=['true', 'false'],
            description='Start Gazebo ros2_control and its controllers.',
        ),
        DeclareLaunchArgument(
            'enable_physical_grasp',
            default_value='false',
            choices=['true', 'false'],
            description='Enable contact sensing and physical pear joints.',
        ),
        DeclareLaunchArgument(
            'enable_camera',
            default_value='false',
            choices=['true', 'false'],
            description='Enable the fixed RGB-D camera and its ROS bridge.',
        ),
        DeclareLaunchArgument(
            'render_engine',
            default_value='ogre',
            choices=['ogre', 'ogre2'],
            description='Gazebo GUI renderer; RGB-D sensors use ogre2 on the server.',
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.dirname(package_share),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': [
                    world_path,
                    ' -r -v 2 --render-engine-gui ',
                    render_engine,
                    ' --render-engine-server ogre2',
                ],
            }.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(Command([
                    'xacro ',
                    urdf_path,
                    ' enable_gz_ros2_control:=',
                    enable_ros2_control,
                    ' enable_physical_grasp:=',
                    enable_physical_grasp,
                    ' enable_camera:=',
                    enable_camera,
                ]), value_type=str),
                'use_sim_time': True,
            }],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_rbt10',
            output='screen',
            arguments=[
                '-name', 'rbt10',
                '-topic', 'robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.0',
                '-Y', '0.0',
            ],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='screen',
            parameters=[{
                'config_file': clock_bridge_config,
                'use_sim_time': True,
            }],
            condition=UnlessCondition(enable_physical_grasp),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='grasp_bridge',
            output='screen',
            parameters=[{
                'config_file': grasp_bridge_config,
                'use_sim_time': True,
            }],
            condition=IfCondition(enable_physical_grasp),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            output='screen',
            parameters=[{
                'config_file': camera_bridge_config,
                'use_sim_time': True,
            }],
            condition=IfCondition(enable_camera),
        ),
        Node(
            package='rbt08pkg',
            executable='grasp_manager.py',
            name='grasp_manager',
            output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(enable_physical_grasp),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name='spawn_gazebo_controllers',
            output='screen',
            arguments=[
                'joint_state_broadcaster',
                'arm_controller',
                'gripper_controller',
                '--controller-manager', '/controller_manager',
                '--controller-manager-timeout', '60',
                '--service-call-timeout', '60',
                '--switch-timeout', '60',
                '--activate-as-group',
            ],
            condition=IfCondition(enable_ros2_control),
        ),
    ])

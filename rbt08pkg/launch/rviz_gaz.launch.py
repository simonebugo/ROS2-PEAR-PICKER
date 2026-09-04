import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_bme_gazebo_basics = get_package_share_directory('rbt08pkg')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Make Gazebo able to find meshes and other resources
    # contained in the rbt08pkg package.
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(pkg_bme_gazebo_basics)
    )

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open RViz.'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='orchard.world.sdf',
        description='Name of the Gazebo world file to load'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='rbt10.urdf',
        description='Name of the URDF description to load'
    )

    # Path to the URDF/Xacro file
    urdf_file_path = PathJoinSubstitution([
        pkg_bme_gazebo_basics,
        "urdf",
        LaunchConfiguration('model')
    ])

    # Launch Gazebo
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            ),
        ),
        launch_arguments={
            'gz_args': [
                PathJoinSubstitution([
                    pkg_bme_gazebo_basics,
                    'worlds',
                    LaunchConfiguration('world')
                ]),
                TextSubstitution(
                    text=' -r -v -v1 --render-engine ogre'
                )
            ]
        }.items()
    )

    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(
                pkg_bme_gazebo_basics,
                'config',
                'config.rviz'
            )
        ],
        condition=IfCondition(
            LaunchConfiguration('rviz')
        ),
        parameters=[
            {'use_sim_time': True},
        ]
    )

    # Spawn the URDF model in Gazebo
    spawn_urdf_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',
            'my_robot',
            '-topic',
            'robot_description',
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.0',
            '-Y',
            '0.0'
        ],
        output='screen',
        parameters=[
            {'use_sim_time': True},
        ]
    )

    # Publish the robot description and TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': Command([
                    'xacro',
                    ' ',
                    urdf_file_path
                ]),
                'use_sim_time': True,
            }
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    # Create launch description
    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)

    # Set Gazebo resource path before starting Gazebo
    launchDescriptionObject.add_action(gz_resource_path)

    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(joint_state_publisher_gui_node)

    return launchDescriptionObject
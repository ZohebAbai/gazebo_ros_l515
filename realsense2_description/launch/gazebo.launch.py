#!/usr/bin/env python3
"""
Launch file to spawn L515 camera model in Gazebo for ROS2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_realsense2_description = get_package_share_directory('realsense2_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Path to URDF xacro file
    default_model_path = os.path.join(pkg_realsense2_description, 'urdf', 'test_l515_camera.urdf.xacro')

    # Launch arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model_path,
        description='Path to robot URDF/xacro file'
    )

    paused_arg = DeclareLaunchArgument(
        'paused',
        default_value='false',
        description='Start Gazebo paused'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI'
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode'
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Run Gazebo in debug mode'
    )

    # Robot description
    robot_description = Command(['xacro ', LaunchConfiguration('model')])

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'paused': LaunchConfiguration('paused'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gui': LaunchConfiguration('gui'),
            'debug': LaunchConfiguration('debug')
        }.items()
    )

    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'l515_camera',
            '-z', '1.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        paused_arg,
        use_sim_time_arg,
        gui_arg,
        headless_arg,
        debug_arg,
        robot_state_publisher_node,
        gazebo,
        spawn_entity
    ])

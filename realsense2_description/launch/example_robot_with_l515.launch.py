#!/usr/bin/env python3
"""
Example launch file for a mobile robot with integrated L515 camera

This demonstrates how to launch a custom robot with the L515 camera in Gazebo.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import conditions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_realsense2_description = get_package_share_directory('realsense2_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Path to example robot URDF
    robot_xacro = os.path.join(
        pkg_realsense2_description,
        'urdf',
        'example_mobile_robot_with_l515.urdf.xacro'
    )

    # Path to RViz config
    rviz_config_path = os.path.join(pkg_realsense2_description, 'rviz', 'urdf.rviz')

    # Launch arguments
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

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    # Robot description
    robot_description = Command(['xacro ', robot_xacro])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gui': LaunchConfiguration('gui')
        }.items()
    )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mobile_robot_with_l515',
            '-z', '0.5'
        ],
        output='screen'
    )

    # RViz (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=conditions.IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        gui_arg,
        rviz_arg,
        robot_state_publisher,
        gazebo,
        spawn_entity,
        rviz_node
    ])

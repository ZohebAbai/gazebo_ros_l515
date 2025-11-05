#!/usr/bin/env python3
"""
Launch file to view and simulate L515 camera in both RViz and Gazebo for ROS2
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

    # Path to URDF xacro file
    default_model_path = os.path.join(pkg_realsense2_description, 'urdf', 'test_l515_camera.urdf.xacro')

    # Path to RViz config
    rviz_config_path = os.path.join(pkg_realsense2_description, 'rviz', 'urdf.rviz')

    # Launch arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model_path,
        description='Path to robot URDF/xacro file'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch RViz GUI'
    )

    # Robot description
    robot_description = Command(['xacro ', LaunchConfiguration('model')])

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0,
            'use_sim_time': True
        }]
    )

    # Include Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_realsense2_description, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'model': LaunchConfiguration('model')
        }.items()
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        gui_arg,
        robot_state_publisher_node,
        gazebo_launch,
        rviz_node
    ])

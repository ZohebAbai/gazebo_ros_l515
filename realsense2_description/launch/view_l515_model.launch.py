#!/usr/bin/env python3
"""
Launch file to view L515 camera model in RViz for ROS2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )

    # Robot description
    robot_description = Command(['xacro ', default_model_path])

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0
        }]
    )

    # Joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_gui': LaunchConfiguration('gui')}]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])

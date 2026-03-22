# Copyright 2025 Zoheb Abai
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch file to view and simulate L515 camera in both RViz and Gazebo for ROS2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_realsense2_description = get_package_share_directory('realsense2_description')

    default_model_path = os.path.join(
        pkg_realsense2_description, 'urdf', 'test_l515_camera.urdf.xacro'
    )

    rviz_config_path = os.path.join(pkg_realsense2_description, 'rviz', 'urdf.rviz')

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

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0,
            'use_sim_time': True,
        }]
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_realsense2_description, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'model': LaunchConfiguration('model'),
        }.items()
    )

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
        rviz_node,
    ])

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

"""Example launch file for a mobile robot with integrated L515 camera."""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import conditions, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_realsense2_description = get_package_share_directory('realsense2_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    robot_xacro = os.path.join(
        pkg_realsense2_description,
        'urdf',
        'example_mobile_robot_with_l515.urdf.xacro'
    )

    rviz_config_path = os.path.join(pkg_realsense2_description, 'rviz', 'urdf.rviz')

    # --- Launch arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    # --- Environment ---
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_realsense2_description, '..')
    )

    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=os.path.join(
            get_package_prefix('realsense_gazebo_plugin'), 'lib'
        )
    )

    # --- Robot description ---
    robot_description = ParameterValue(
        Command(['xacro ', robot_xacro]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen'
    )

    # --- Gazebo Harmonic ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # --- Spawn robot ---
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mobile_robot_with_l515',
            '-z', '0.5'
        ],
        output='screen'
    )

    # --- Bridges (robot_camera namespace matches topics_ns in URDF) ---
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    camera_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
            '/robot_camera/color/image_raw',
            '/robot_camera/depth/image_raw',
            '/robot_camera/infra/image_raw',
        ],
        output='screen'
    )

    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/robot_camera/color/camera_info'
            '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/robot_camera/depth/camera_info'
            '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/robot_camera/infra/camera_info'
            '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    pointcloud_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/robot_camera/depth/color/points'
            '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        output='screen'
    )

    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        output='screen'
    )

    # --- RViz (conditional) ---
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
        gz_resource_path,
        gz_plugin_path,
        use_sim_time_arg,
        rviz_arg,
        robot_state_publisher,
        gazebo,
        clock_bridge,
        spawn_entity,
        camera_image_bridge,
        camera_info_bridge,
        pointcloud_bridge,
        cmd_vel_bridge,
        odom_bridge,
        rviz_node,
    ])

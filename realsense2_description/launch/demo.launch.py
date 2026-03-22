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

"""Demo launch: L515 camera in a colorful scene — color, depth, and point cloud."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_realsense2_description = get_package_share_directory('realsense2_description')
    pkg_realsense_gazebo_plugin = get_package_share_directory('realsense_gazebo_plugin')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    model_path = os.path.join(
        pkg_realsense2_description, 'urdf', 'test_l515_camera.urdf.xacro'
    )
    world_path = os.path.join(pkg_realsense2_description, 'worlds', 'l515_demo.sdf')
    rviz_config_path = os.path.join(pkg_realsense2_description, 'rviz', 'demo.rviz')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_realsense2_description, '..')
    )

    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=os.path.join(pkg_realsense_gazebo_plugin, 'lib')
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    robot_description = ParameterValue(
        Command(['xacro ', model_path]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'l515_camera',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.2',
        ],
        output='screen'
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Gazebo sensor topic layout (set by <topic> in _l515.gazebo.xacro):
    #   /camera/color         gz.msgs.Image       -> /camera/color/image_raw
    #   /camera/color/camera_info gz.msgs.CameraInfo -> /camera/color/camera_info
    #   /camera/infra         gz.msgs.Image       -> /camera/infra/image_raw
    #   /camera/infra/camera_info gz.msgs.CameraInfo -> /camera/infra/camera_info
    #   /camera/depth         gz.msgs.Image       -> /camera/depth/image_raw
    #   /camera/depth/points  gz.msgs.PointCloudPacked -> /camera/depth/color/points
    #   /camera/depth/camera_info gz.msgs.CameraInfo -> /camera/depth/camera_info

    color_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/color'],
        remappings=[('/camera/color', '/camera/color/image_raw')],
        output='screen'
    )

    infra_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/infra'],
        remappings=[('/camera/infra', '/camera/infra/image_raw')],
        output='screen'
    )

    depth_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/depth'],
        remappings=[('/camera/depth', '/camera/depth/image_raw')],
        output='screen'
    )

    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/infra/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    pointcloud_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[('/camera/depth/points', '/camera/depth/color/points')],
        output='screen'
    )

    # Auto-unpause Gazebo 8 seconds after launch so sensors start rendering
    unpause_gazebo = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service',
                    '-s', '/world/l515_demo/control',
                    '--reqtype', 'gz.msgs.WorldControl',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000',
                    '--req', 'pause: false',
                ],
                output='screen'
            )
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        gz_plugin_path,
        rviz_arg,
        robot_state_publisher_node,
        gazebo,
        clock_bridge,
        spawn_entity,
        color_image_bridge,
        infra_image_bridge,
        depth_image_bridge,
        camera_info_bridge,
        pointcloud_bridge,
        unpause_gazebo,
        rviz_node,
    ])

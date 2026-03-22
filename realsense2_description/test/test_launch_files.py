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

"""
Tests for realsense2_description launch files.

Validates structural correctness of all launch files without starting
Gazebo (no display required). Checks:
- No Gazebo Classic (gazebo_ros) references
- Bridge nodes use correct argument format
- All required camera topics are bridged
- Clock bridge is present and correctly typed
"""

import os

from ament_index_python.packages import get_package_share_directory
import pytest


@pytest.fixture(scope='module')
def launch_dir():
    return os.path.join(
        get_package_share_directory('realsense2_description'), 'launch'
    )


def _read(launch_dir, filename):
    with open(os.path.join(launch_dir, filename)) as f:
        return f.read()


# ---------------------------------------------------------------------------
# gazebo.launch.py
# ---------------------------------------------------------------------------

class TestGazeboLaunch:

    def test_no_gazebo_classic(self, launch_dir):
        content = _read(launch_dir, 'gazebo.launch.py')
        assert 'gazebo_ros' not in content, \
            'gazebo.launch.py must not reference gazebo_ros (Gazebo Classic)'
        assert 'spawn_entity.py' not in content, \
            'spawn_entity.py is Gazebo Classic; use ros_gz_sim:create'

    def test_uses_ros_gz_sim_create(self, launch_dir):
        content = _read(launch_dir, 'gazebo.launch.py')
        assert "executable='create'" in content or 'ros_gz_sim' in content, \
            'gazebo.launch.py must spawn via ros_gz_sim:create'

    def test_clock_bridge(self, launch_dir):
        content = _read(launch_dir, 'gazebo.launch.py')
        assert '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' in content, \
            'Clock bridge must use correct type mapping'

    def test_all_camera_info_topics_bridged(self, launch_dir):
        content = _read(launch_dir, 'gazebo.launch.py')
        for stream in ('color', 'depth', 'infra'):
            assert f'/camera/{stream}/camera_info' in content, \
                f'camera_info not bridged for stream: {stream}'

    def test_depth_image_bridged(self, launch_dir):
        content = _read(launch_dir, 'gazebo.launch.py')
        assert '/camera/depth' in content, \
            'Depth image must be bridged in gazebo.launch.py'

    def test_color_image_bridged(self, launch_dir):
        content = _read(launch_dir, 'gazebo.launch.py')
        assert '/camera/color' in content, \
            'Color image must be bridged in gazebo.launch.py'


# ---------------------------------------------------------------------------
# example_robot_with_l515.launch.py
# ---------------------------------------------------------------------------

class TestExampleRobotLaunch:

    def test_no_gazebo_classic_spawn(self, launch_dir):
        content = _read(launch_dir, 'example_robot_with_l515.launch.py')
        assert 'spawn_entity.py' not in content, \
            'example launch must not use spawn_entity.py (Gazebo Classic)'

    def test_no_gazebo_ros_package(self, launch_dir):
        content = _read(launch_dir, 'example_robot_with_l515.launch.py')
        assert 'gazebo_ros' not in content, \
            'example launch must not reference gazebo_ros (Gazebo Classic)'

    def test_uses_ros_gz_sim_create(self, launch_dir):
        content = _read(launch_dir, 'example_robot_with_l515.launch.py')
        assert "package='ros_gz_sim'" in content, \
            'example launch must use ros_gz_sim for spawning'
        assert "executable='create'" in content, \
            'example launch must use ros_gz_sim:create'

    def test_uses_gz_sim_launch(self, launch_dir):
        content = _read(launch_dir, 'example_robot_with_l515.launch.py')
        assert 'gz_sim.launch.py' in content, \
            'example launch must include gz_sim.launch.py from ros_gz_sim'

    def test_clock_bridge_present(self, launch_dir):
        content = _read(launch_dir, 'example_robot_with_l515.launch.py')
        assert '/clock' in content, \
            'example launch must bridge /clock for simulation time'

    def test_bridge_nodes_present(self, launch_dir):
        content = _read(launch_dir, 'example_robot_with_l515.launch.py')
        assert 'ros_gz_bridge' in content, \
            'example launch must include ros_gz_bridge'
        assert 'ros_gz_image' in content, \
            'example launch must include ros_gz_image'

    def test_robot_camera_namespace_bridged(self, launch_dir):
        content = _read(launch_dir, 'example_robot_with_l515.launch.py')
        assert 'robot_camera' in content, \
            'robot_camera namespace must be bridged (matches topics_ns in URDF)'

    def test_cmd_vel_bridge_present(self, launch_dir):
        content = _read(launch_dir, 'example_robot_with_l515.launch.py')
        assert 'cmd_vel' in content, \
            'cmd_vel must be bridged to allow robot movement'


# ---------------------------------------------------------------------------
# Cross-cutting: no Gazebo Classic in any launch file
# ---------------------------------------------------------------------------

class TestNoGazeboClassicAnywhere:

    def test_all_launch_files_harmonic_only(self, launch_dir):
        classic_markers = ['gazebo_ros', 'spawn_entity.py', 'libgazebo_ros']
        for fname in os.listdir(launch_dir):
            if not fname.endswith('.py'):
                continue
            content = _read(launch_dir, fname)
            for marker in classic_markers:
                assert marker not in content, (
                    f'{fname} contains Gazebo Classic reference: {marker!r}'
                )

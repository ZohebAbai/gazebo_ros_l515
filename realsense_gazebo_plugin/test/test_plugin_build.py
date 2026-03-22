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
Tests for realsense_gazebo_plugin build artifacts.

Verifies the plugin .so was built correctly, is a valid ELF shared library,
exports Gazebo plugin symbols, and does NOT export image_transport symbols
(regression guard against re-introducing dead code).
"""

import os
import subprocess

from ament_index_python.packages import get_package_prefix
import pytest


@pytest.fixture(scope='module')
def lib_path():
    prefix = get_package_prefix('realsense_gazebo_plugin')
    # Resolve symlinks — colcon --symlink-install creates .so symlinks pointing
    # to the build directory; os.path.realpath gives us the actual ELF file.
    return os.path.realpath(
        os.path.join(prefix, 'lib', 'librealsense_gazebo_plugin.so')
    )


def test_plugin_library_exists(lib_path):
    """The .so file must exist after a successful build."""
    assert os.path.exists(lib_path), (
        f'Plugin library not found at: {lib_path}\n'
        'Run: colcon build --packages-select realsense_gazebo_plugin'
    )


def test_plugin_library_is_shared_object(lib_path):
    """Verify the .so is a valid ELF shared library."""
    result = subprocess.run(
        ['file', lib_path], capture_output=True, text=True, check=True
    )
    assert 'ELF' in result.stdout and 'shared object' in result.stdout, (
        f'Library is not a valid ELF shared object:\n{result.stdout}'
    )


def test_plugin_exports_gz_symbols(lib_path):
    """Plugin must export Gazebo plugin registration symbols (from GZ_ADD_PLUGIN)."""
    result = subprocess.run(
        ['nm', '-D', lib_path], capture_output=True, text=True, check=True
    )
    symbols = result.stdout
    assert (
        'RealSensePluginHarmonic' in symbols or
        'gz_plugin' in symbols.lower()
    ), 'Plugin must export Gazebo plugin registration symbols'


def test_no_image_transport_symbols(lib_path):
    """image_transport must NOT be linked — it was dead code that was removed."""
    result = subprocess.run(
        ['nm', '-D', lib_path], capture_output=True, text=True, check=True
    )
    assert 'image_transport' not in result.stdout, (
        'image_transport symbols found in the plugin library. '
        'Dead code was re-introduced — remove image_transport dependency.'
    )


def test_no_camera_info_manager_symbols(lib_path):
    """camera_info_manager must NOT be linked — it was dead code that was removed."""
    result = subprocess.run(
        ['nm', '-D', lib_path], capture_output=True, text=True, check=True
    )
    assert 'camera_info_manager' not in result.stdout, (
        'camera_info_manager symbols found in the plugin library. '
        'Dead code was re-introduced — remove camera_info_manager dependency.'
    )

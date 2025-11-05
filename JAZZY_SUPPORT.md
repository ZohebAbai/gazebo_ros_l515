# ROS2 Jazzy + Gazebo Harmonic Support

This document explains the support for ROS2 Jazzy with the new Gazebo (Gazebo Harmonic).

## What Changed in Jazzy?

**ROS2 Jazzy** (released May 2024) moved from **Gazebo Classic** to the **new Gazebo** (formerly called Ignition Gazebo, now just "Gazebo" or "Gazebo Sim").

### Key Differences:

| Aspect | Humble/Iron (Old) | Jazzy (New) |
|--------|------------------|-------------|
| Simulator | Gazebo Classic (gazebo11) | New Gazebo (Gazebo Harmonic) |
| Package Name | `gazebo_ros` | `ros_gz_sim` |
| CMake Package | `gazebo_dev` | `gz-sim8` or `gz-sim7` |
| Binary | `gazebo` | `gz sim` |
| Version | 11.x | 8.x (Harmonic) |

## Installation for Jazzy

### 1. Install ROS2 Jazzy

```bash
# Add ROS2 apt repository (if not done)
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-jazzy-desktop
```

### 2. Install Gazebo Harmonic

```bash
# Install Gazebo Harmonic
sudo apt install gz-harmonic

# Verify installation
gz sim --version
# Should show: Gazebo Sim, version 8.x.x
```

### 3. Install ROS-Gazebo Bridge

```bash
# Install ROS2-Gazebo integration packages
sudo apt install \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-interfaces
```

### 4. Install Other Dependencies

```bash
sudo apt install \
    ros-jazzy-image-transport \
    ros-jazzy-camera-info-manager \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-rviz2
```

## Building for Jazzy

```bash
# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Create workspace
mkdir -p ~/ros2_jazzy_ws/src
cd ~/ros2_jazzy_ws/src

# Clone repository
git clone -b claude/ros2-port-issue-3-011CUN35QEciLKb4TC1M95Jo \
    https://github.com/ZohebAbai/gazebo_ros_l515.git

cd ~/ros2_jazzy_ws

# Build (will automatically detect Jazzy and use new Gazebo)
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### Build Output

When building for Jazzy, you should see:

```
-- Detected ROS2 jazzy - Using new Gazebo (Harmonic)
-- Found Gazebo Harmonic (gz-sim8)
```

## Running with Jazzy

### Launch Command (Same as Before!)

```bash
source ~/ros2_jazzy_ws/install/setup.bash
ros2 launch realsense2_description gazebo.launch.py
```

**Note:** The launch command is the same! The package automatically handles the differences internally.

### Using New Gazebo Commands

You can also use the new Gazebo commands:

```bash
# Launch Gazebo Harmonic directly
gz sim

# List topics (new Gazebo topics)
gz topic -l

# Echo topic
gz topic -e -t /some/topic
```

## Differences from Gazebo Classic

### 1. Command Line

**Gazebo Classic:**
```bash
gazebo
gazebo --verbose
```

**New Gazebo:**
```bash
gz sim
gz sim -v 4  # verbose level 4
```

### 2. Topics and Communication

**Gazebo Classic** used custom Gazebo transport.
**New Gazebo** uses Gazebo Transport (still different from ROS).

The `ros_gz_bridge` handles translation between ROS2 and Gazebo topics.

### 3. Plugin Loading

The plugin loading mechanism is slightly different, but the package handles this automatically through CMake detection.

## Conda Users with Jazzy

If you're using conda for ROS2:

```bash
conda activate ros2

# Install ROS2 Jazzy
conda install ros-jazzy-desktop-full

# Install new Gazebo support
conda install ros-jazzy-ros-gz-sim

# For Gazebo itself, you might need to use system packages
# as conda may not have gz-harmonic yet
```

## Known Issues and Limitations

### Issue 1: Plugin May Need Updates

⚠️ **The C++ plugin code was written for Gazebo Classic.** While the ROS2-Gazebo bridge provides compatibility, some features might not work perfectly with Gazebo Harmonic.

**Status:** Needs testing with real Jazzy installation.

### Issue 2: Performance Differences

The new Gazebo (Harmonic) has different performance characteristics than Gazebo Classic. You may notice:
- Different rendering performance
- Different physics simulation speed
- Different memory usage

### Issue 3: SDF Format

Gazebo Harmonic uses **SDFormat specification 1.10+**, which has some differences from older versions used in Gazebo Classic.

## Troubleshooting

### Error: "Could not find gz-sim"

```bash
# Check if Gazebo is installed
gz sim --version

# If not found, install:
sudo apt install gz-harmonic
```

### Error: "Package 'ros_gz_sim' not found"

```bash
# Install the ROS-Gazebo bridge
sudo apt install ros-jazzy-ros-gz-sim
```

### Build Error: "gazebo_dev not found"

This is expected! The package should automatically detect Jazzy and NOT look for `gazebo_dev`.

**Check:**
1. Is `ROS_DISTRO` set correctly?
   ```bash
   echo $ROS_DISTRO
   # Should output: jazzy
   ```

2. Source your ROS2 installation:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

### Plugin Not Loading

**If you see:** "Could not load plugin"

1. Check plugin path:
   ```bash
   echo $GZ_SIM_RESOURCE_PATH
   echo $GZ_SIM_SYSTEM_PLUGIN_PATH
   ```

2. The plugin should be in:
   ```
   ~/ros2_jazzy_ws/install/realsense_gazebo_plugin/lib/
   ```

## Testing Your Build

### Quick Test

```bash
# Test if package is found
ros2 pkg list | grep realsense

# Should show:
# realsense2_description
# realsense_gazebo_plugin
```

### Full Test

```bash
# Launch with Gazebo
ros2 launch realsense2_description gazebo.launch.py

# In another terminal, check topics
source ~/ros2_jazzy_ws/install/setup.bash
ros2 topic list

# Should see camera topics
```

## Reporting Issues

If you encounter issues specific to Jazzy + Gazebo Harmonic:

1. **Verify your setup:**
   ```bash
   ros2 --version
   gz sim --version
   echo $ROS_DISTRO
   ```

2. **Build with verbose output:**
   ```bash
   colcon build --event-handlers console_direct+
   ```

3. **Report with:**
   - ROS2 version
   - Gazebo version
   - Build output
   - Runtime errors
   - Launch command used

## Future Work

For full Gazebo Harmonic support, the following may need updates:

- [ ] Test all camera features with real Jazzy installation
- [ ] Update plugin for any Gazebo API changes
- [ ] Optimize for new Gazebo performance characteristics
- [ ] Update SDF descriptions if needed
- [ ] Add Jazzy-specific examples

## References

- [ROS2 Jazzy Release](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)
- [ros_gz packages](https://github.com/gazebosim/ros_gz)
- [Migration Guide from Classic to New Gazebo](https://gazebosim.org/docs/latest/migration_from_gazebo_classic)

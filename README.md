# Gazebo ROS L515 - ROS2 Jazzy + Gazebo Harmonic

[![ROS versions](https://img.shields.io/badge/ROS-Jazzy-brightgreen)](https://docs.ros.org)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-blue)](https://gazebosim.org)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-orange)](https://ubuntu.com)

**Intel RealSense L515 sensor simulation for ROS2 Jazzy with Gazebo Harmonic**

![Gazebo L515](gazebo_l515.png)

## ⚠️ Important Notice - Version Support

**This version ONLY supports:**
- ✅ **ROS2 Jazzy** (and Rolling)
- ✅ **Gazebo Harmonic** (gz-sim8)
- ✅ **Ubuntu 24.04**

**Support DISCONTINUED for:**
- ❌ **ROS 1** (Melodic, Noetic, etc.) - Not supported
- ❌ **Older ROS2** (Foxy, Galactic, Humble, Iron) - Not supported
- ❌ **Gazebo Classic** (gazebo11, gazebo9, etc.) - Not supported
- ❌ **Older Ubuntu** (20.04, 22.04) - Not supported

> **Note:** If you need support for ROS 1 or older ROS2 distributions with Gazebo Classic, please use an earlier version of this repository or contact the maintainer.

## Requirements

- **OS:** Ubuntu 24.04 (Noble Numbat)
- **ROS:** ROS2 Jazzy Jalisco
- **Gazebo:** Gazebo Harmonic (gz-sim8)

## Quick Start (Ubuntu 24.04)

Run this single command to install everything:

```bash
curl -sSL https://raw.githubusercontent.com/ZohebAbai/gazebo_ros_l515/claude/ros2-port-issue-3-011CUN35QEciLKb4TC1M95Jo/install_ubuntu24.sh | bash
```

Or follow the manual installation steps below.

---

## Manual Installation

### 1. Install ROS2 Jazzy

```bash
# Ensure Ubuntu 24.04
lsb_release -a  # Should show Ubuntu 24.04

# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy
sudo apt update
sudo apt install ros-jazzy-desktop -y
```

### 2. Install Gazebo Harmonic

```bash
# Install Gazebo Harmonic
sudo apt install gz-harmonic -y

# Verify installation
gz sim --version
# Output should be: Gazebo Sim, version 8.x.x
```

### 3. Install ROS-Gazebo Bridge and Dependencies

```bash
# Install ros_gz packages
sudo apt install \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-image-transport \
    ros-jazzy-camera-info-manager \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-rviz2 -y
```

### 4. Source ROS2

```bash
# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Add to bashrc (optional, but recommended)
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

---

## Build the Package

### 1. Create Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone Repository

```bash
# Clone the repository
git clone -b claude/ros2-port-issue-3-011CUN35QEciLKb4TC1M95Jo https://github.com/ZohebAbai/gazebo_ros_l515.git

cd ~/ros2_ws
```

### 3. Build

```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

**Expected output:**
```
Starting >>> realsense2_description
Starting >>> realsense_gazebo_plugin
Finished <<< realsense2_description [Xs]
Finished <<< realsense_gazebo_plugin [Xs]

Summary: 2 packages finished [Xs]
```

---

## Launch

### Launch L515 in Gazebo

```bash
# Make sure workspace is sourced
source ~/ros2_ws/install/setup.bash

# Launch Gazebo with L515 camera
ros2 launch realsense2_description gazebo.launch.py
```

### Launch with RViz

```bash
# Launch Gazebo + RViz
ros2 launch realsense2_description view_l515_model_rviz_gazebo.launch.py
```

### Launch Example Robot with L515

```bash
# Launch example mobile robot with L515 camera
ros2 launch realsense2_description example_robot_with_l515.launch.py
```

---

## Verify Topics

In a **new terminal**:

```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# List topics
ros2 topic list
```

**Expected topics:**
```
/camera/color/camera_info
/camera/color/image_raw
/camera/depth/camera_info
/camera/depth/image_rect_raw
/camera/depth/color/points
/camera/infra/camera_info
/camera/infra/image_raw
/clock
/parameter_events
/rosout
/tf
/tf_static
```

### Check Topic Data

```bash
# Check image publishing rate
ros2 topic hz /camera/color/image_raw
# Should show: ~30 Hz

# View camera info
ros2 topic echo /camera/color/camera_info --once

# Echo color image (shows data structure)
ros2 topic echo /camera/color/image_raw --once
```

---

## Visualize in RViz2

```bash
# If not already launched with RViz
rviz2
```

**In RViz2:**
1. Click **Add** → **Image**
2. Set topic to `/camera/color/image_raw`
3. Click **Add** → **PointCloud2**
4. Set topic to `/camera/depth/color/points`
5. Set Fixed Frame to `camera_link`

---

## Adding L515 to Your Robot

Want to add the L515 to your own robot? It's easy!

### Quick Example

```xml
<!-- In your robot's URDF/xacro file -->
<xacro:include filename="$(find realsense2_description)/urdf/_l515.urdf.xacro" />

<xacro:sensor_l515
    name="my_camera"
    topics_ns="my_camera"
    parent="base_link"
    use_nominal_extrinsics="true"
    publish_pointcloud="true">
    <origin xyz="0.1 0 0.2" rpy="0 0 0" />
</xacro:sensor_l515>
```

**For detailed integration guide:** See [INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md)

---

## Testing

For comprehensive testing procedures, see [TESTING_GUIDE.md](TESTING_GUIDE.md)

**Quick test:**
```bash
# Check packages
ros2 pkg list | grep realsense

# Launch and verify
ros2 launch realsense2_description gazebo.launch.py
```

---

## Troubleshooting

### "gz sim: command not found"

```bash
sudo apt install gz-harmonic
```

### "Package 'ros_gz_sim' not found"

```bash
sudo apt install ros-jazzy-ros-gz-sim
```

### Build fails

```bash
# Clean and rebuild
cd ~/ros2_ws
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --event-handlers console_direct+
```

### Gazebo doesn't open

```bash
# Check Gazebo installation
gz sim --version

# Try running Gazebo standalone
gz sim
```

### No camera topics

```bash
# Check if nodes are running
ros2 node list

# Check for errors in launch terminal
# Camera plugin should load automatically
```

---

## Commands Summary

```bash
# Install (run once)
sudo apt update
sudo apt install ros-jazzy-desktop gz-harmonic ros-jazzy-ros-gz-sim -y

# Build
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone -b claude/ros2-port-issue-3-011CUN35QEciLKb4TC1M95Jo https://github.com/ZohebAbai/gazebo_ros_l515.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# Launch
source ~/ros2_ws/install/setup.bash
ros2 launch realsense2_description gazebo.launch.py
```

---

## System Requirements

- **CPU:** Modern multi-core processor (Intel i5 or better)
- **RAM:** 8GB minimum, 16GB recommended
- **GPU:** Dedicated GPU recommended for Gazebo
- **Disk:** ~10GB free space

---

## Documentation

- 📖 [INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md) - How to integrate L515 into your robot
- 🧪 [TESTING_GUIDE.md](TESTING_GUIDE.md) - Complete testing procedures
- 🚀 [JAZZY_SUPPORT.md](JAZZY_SUPPORT.md) - Jazzy-specific information

---

## Demo

[Video](https://youtu.be/KoQNH7YahN8)

---

## Acknowledgement

This repository was created to fulfill the need for a package to use simulated L515 sensor in Gazebo.
The repository is a combined modification of following repositories:
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros)
- [realsense_gazebo_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)
- [realsense-ros-gazebo](https://github.com/rickstaa/realsense-ros-gazebo/)

---

## Contributing

Contributions are welcome! Please open an issue or pull request on GitHub.

## License

Apache 2.0

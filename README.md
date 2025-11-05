# Gazebo ROS L515

[![ROS versions](https://img.shields.io/badge/ROS%20versions-ROS2-brightgreen)](https://docs.ros.org)
[![Contributions](https://img.shields.io/badge/contributions-welcome-orange.svg)](contributing.md)

**This repository contains all the code required for implementation of simulated RealSense L515 sensor with ROS2 in [Gazebo](https://gazebosim.org).**

![Gazebo L515](gazebo_l515.png)

*If you require gazebo implementation for other Intel Realsense cameras (ex: D435) please check the repositories mentioned in the acknowledgement section.*

## ROS2 Port

This package has been ported to ROS2 and supports multiple ROS2 distributions and Gazebo versions:

**Supported ROS2 Distributions:**
- ✅ **Foxy** / **Galactic** / **Humble** / **Iron** - Uses Gazebo Classic (gazebo11)
- ✅ **Jazzy** / **Rolling** - Uses new Gazebo (Gazebo Harmonic/Garden)

**Key Features:**
- Updated build system to `ament_cmake` with automatic Gazebo detection
- Converted all C++ code to use `rclcpp` and ROS2 APIs
- Converted launch files to Python format
- Automatic detection of Gazebo Classic vs New Gazebo based on ROS distro

## Install Dependencies

### For ROS2 Humble, Iron (and earlier) - Gazebo Classic

```shell
# Install Gazebo Classic dependencies
sudo apt install \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-xacro
```

### For ROS2 Jazzy (and later) - Gazebo Harmonic

```shell
# Install new Gazebo (Harmonic) dependencies
sudo apt install \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    gz-harmonic
```

**Note:** The package automatically detects which Gazebo version to use based on your ROS_DISTRO environment variable.

## Build the Package
If you already have a ROS2 workspace skip this step, else:
```shell
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

**Clone the repository and build the package**
```shell
git clone https://github.com/zohebabai/gazebo_ros_l515.git
cd ~/ros2_ws
colcon build --symlink-install
```

If not done already, add this to your `~/.bashrc` file for sourcing your workspace:
```shell
source ~/ros2_ws/install/setup.bash
```

Or source it manually in your current terminal:
```shell
source ~/ros2_ws/install/setup.bash
```

## Launch the ROS2 Package

Launch with both Gazebo and RViz:
```shell
ros2 launch realsense2_description view_l515_model_rviz_gazebo.launch.py
```

Launch only Gazebo simulation:
```shell
ros2 launch realsense2_description gazebo.launch.py
```

Launch only RViz visualization:
```shell
ros2 launch realsense2_description view_l515_model.launch.py
```

## ROS2 Topics

After launching, you can view the available topics:
```shell
ros2 topic list
```

Expected topics include:
- `/camera/color/image_raw` - Color camera image
- `/camera/color/camera_info` - Color camera info
- `/camera/depth/image_raw` - Depth image
- `/camera/depth/camera_info` - Depth camera info
- `/camera/depth/points` - Point cloud data (if enabled)
- `/camera/infrared/image_raw` - Infrared image
- `/camera/infrared/camera_info` - Infrared camera info

## Adding L515 to Your Existing Robot

Want to integrate the L515 camera into your own robot? It's easy!

**Quick Example:**
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

**For detailed instructions, examples, and troubleshooting, see:**
📖 **[INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md)**

The integration guide covers:
- Step-by-step integration instructions
- Parameter explanations
- Multiple camera examples
- Common use cases (mobile robots, robot arms, etc.)
- Troubleshooting tips

**Example Robot:**
We provide a complete example of a mobile robot with integrated L515:
```shell
ros2 launch realsense2_description example_robot_with_l515.launch.py
```

## Demo
[Video](https://youtu.be/KoQNH7YahN8)

## Acknowledgement
The repository was created to fulfill the need for a package to use simulated L515 sensor in Gazebo.
The repository is a combined modification of following repositories:
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros)
- [realsense_gazebo_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)
- [realsense-ros-gazebo](https://github.com/rickstaa/realsense-ros-gazebo/)

# Complete Commands for Ubuntu 24.04 + ROS2 Jazzy + Gazebo Harmonic

**⚠️ IMPORTANT:** This package ONLY supports ROS2 Jazzy + Gazebo Harmonic on Ubuntu 24.04.
- **No ROS 1 support** (Melodic, Noetic, etc.)
- **No older ROS2 support** (Foxy, Galactic, Humble, Iron)
- **No Gazebo Classic support** (gazebo11, etc.)

See [VERSION_SUPPORT.md](VERSION_SUPPORT.md) for details on version compatibility.

---

Copy and paste these commands into your Ubuntu 24.04 terminal.

## Step 1: Install ROS2 Jazzy

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Set locale
sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repository
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy
sudo apt update
sudo apt install ros-jazzy-desktop -y
```

## Step 2: Install Gazebo Harmonic

```bash
# Install Gazebo Harmonic
sudo apt install gz-harmonic -y

# Verify
gz sim --version
```

## Step 3: Install Dependencies

```bash
# Install all required packages
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
    ros-jazzy-rviz2 \
    python3-colcon-common-extensions -y
```

## Step 4: Setup Environment

```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash

# Add to bashrc (run once)
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Step 5: Build Package

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone -b claude/ros2-port-issue-3-011CUN35QEciLKb4TC1M95Jo https://github.com/ZohebAbai/gazebo_ros_l515.git

# Build
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Add to bashrc (optional)
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Step 6: Launch

```bash
# Terminal 1: Launch Gazebo with L515
source ~/ros2_ws/install/setup.bash
ros2 launch realsense2_description gazebo.launch.py
```

```bash
# Terminal 2: Check topics
source ~/ros2_ws/install/setup.bash
ros2 topic list
ros2 topic hz /camera/color/image_raw
```

---

## All-in-One Command (Installation Only)

Run this to install everything:

```bash
# Install ROS2 Jazzy + Gazebo Harmonic + Dependencies
sudo apt update && \
sudo apt install -y locales software-properties-common curl && \
sudo locale-gen en_US en_US.UTF-8 && \
sudo add-apt-repository universe -y && \
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
sudo apt update && \
sudo apt install -y \
    ros-jazzy-desktop \
    gz-harmonic \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-camera-info-manager \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-rviz2 \
    python3-colcon-common-extensions && \
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
source /opt/ros/jazzy/setup.bash && \
echo "✅ Installation complete! ROS2 Jazzy + Gazebo Harmonic ready."
```

Then build the package:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src && \
git clone -b claude/ros2-port-issue-3-011CUN35QEciLKb4TC1M95Jo https://github.com/ZohebAbai/gazebo_ros_l515.git && \
cd ~/ros2_ws && \
source /opt/ros/jazzy/setup.bash && \
colcon build --symlink-install && \
echo "✅ Build complete! Source workspace: source ~/ros2_ws/install/setup.bash"
```

Then launch:

```bash
source ~/ros2_ws/install/setup.bash && \
ros2 launch realsense2_description gazebo.launch.py
```

---

## Verification Commands

```bash
# Check ROS2
ros2 --version

# Check Gazebo
gz sim --version

# Check packages
ros2 pkg list | grep realsense

# Check topics (after launch)
ros2 topic list

# Check image rate
ros2 topic hz /camera/color/image_raw
```

---

## Quick Reference

| Command | Purpose |
|---------|---------|
| `source /opt/ros/jazzy/setup.bash` | Source ROS2 |
| `source ~/ros2_ws/install/setup.bash` | Source workspace |
| `colcon build --symlink-install` | Build packages |
| `ros2 launch realsense2_description gazebo.launch.py` | Launch L515 in Gazebo |
| `ros2 topic list` | List all topics |
| `gz sim` | Launch Gazebo Harmonic |
| `rviz2` | Launch RViz2 |

---

## Troubleshooting

### Command not found errors
```bash
# Make sure ROS2 is sourced
source /opt/ros/jazzy/setup.bash
```

### Build errors
```bash
# Clean and rebuild
cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install --event-handlers console_direct+
```

### Gazebo not opening
```bash
# Check Gazebo installation
gz sim --version

# Try standalone
gz sim
```

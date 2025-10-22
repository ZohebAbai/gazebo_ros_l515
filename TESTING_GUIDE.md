# Testing Guide for ROS2 Port

This guide will help you verify that the ROS2 port of gazebo_ros_l515 is working correctly.

## Prerequisites

Before testing, ensure you have:
- ROS2 installed (Foxy, Galactic, Humble, or Iron)
- Gazebo Classic (gazebo11 or later)
- A clean workspace

## Step-by-Step Testing Procedure

### 1. Set Up the Workspace

```bash
# Create a new ROS2 workspace
mkdir -p ~/test_l515_ws/src
cd ~/test_l515_ws/src

# Clone the repository (use the branch you want to test)
git clone -b claude/ros2-port-issue-3-011CUN35QEciLKb4TC1M95Jo https://github.com/ZohebAbai/gazebo_ros_l515.git

# Go back to workspace root
cd ~/test_l515_ws
```

### 2. Install Dependencies

```bash
# Install ROS2 dependencies
sudo apt update
sudo apt install -y \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2

# Check if rosdep is initialized
rosdep update

# Install any additional dependencies
cd ~/test_l515_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Package

```bash
cd ~/test_l515_ws

# Build with verbose output to catch any errors
colcon build --symlink-install --event-handlers console_direct+

# Check the build output for any errors or warnings
```

**Expected Output:**
```
Starting >>> realsense2_description
Starting >>> realsense_gazebo_plugin
Finished <<< realsense2_description [Xs]
Finished <<< realsense_gazebo_plugin [Xs]

Summary: 2 packages finished [Xs]
```

**If build fails, check:**
- Missing dependencies
- Compiler errors (indicates code issues)
- CMake configuration errors

### 4. Source the Workspace

```bash
source ~/test_l515_ws/install/setup.bash

# Verify packages are found
ros2 pkg list | grep realsense
```

**Expected Output:**
```
realsense2_description
realsense_gazebo_plugin
```

### 5. Test 1: Launch L515 in Gazebo Only

```bash
# Launch Gazebo with the L515 camera
ros2 launch realsense2_description gazebo.launch.py
```

**What to verify:**
1. ✅ Gazebo launches without errors
2. ✅ L515 camera model appears in Gazebo world (should be floating at z=1.0)
3. ✅ Camera looks correct (cylindrical shape with actual L515 mesh)
4. ✅ No red error messages in terminal

**In a new terminal, check topics:**
```bash
source ~/test_l515_ws/install/setup.bash
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
/robot_description
/rosout
/tf
/tf_static
```

### 6. Test 2: Verify Camera Data is Publishing

```bash
# In a new terminal
source ~/test_l515_ws/install/setup.bash

# Check color image topic
ros2 topic hz /camera/color/image_raw
```

**Expected Output:**
```
average rate: ~30.000
    min: 0.033s max: 0.033s std dev: 0.00001s window: 30
```

**Test other topics:**
```bash
# Depth image
ros2 topic hz /camera/depth/image_rect_raw

# Camera info
ros2 topic hz /camera/color/camera_info

# Point cloud (may be 0 if no subscribers initially)
ros2 topic hz /camera/depth/color/points
```

### 7. Test 3: Verify Image Data Content

```bash
# Check if images contain actual data (not zeros)
ros2 topic echo /camera/color/image_raw --once
```

**What to verify:**
- `height: 1080`
- `width: 1920`
- `encoding: "rgb8"`
- `data:` should contain non-zero values (actual image data)

### 8. Test 4: Visualize in RViz

```bash
# Launch with both Gazebo and RViz
ros2 launch realsense2_description view_l515_model_rviz_gazebo.launch.py
```

**What to verify in RViz:**
1. ✅ Robot model appears (camera with TF frames)
2. ✅ TF tree is correct
3. ✅ Add Image display → set topic to `/camera/color/image_raw`
4. ✅ Image appears and shows actual camera view from Gazebo
5. ✅ Add PointCloud2 display → set topic to `/camera/depth/color/points`
6. ✅ Point cloud appears

**Check TF tree:**
```bash
ros2 run tf2_tools view_frames
# Open frames.pdf to see the TF tree
```

**Expected frames:**
```
base_link
└── camera_bottom_screw_frame
    └── camera_link
        ├── camera_depth_frame
        │   └── camera_depth_optical_frame
        ├── camera_color_frame
        │   └── camera_color_optical_frame
        └── camera_infra_frame
            └── camera_infra_optical_frame
```

### 9. Test 5: Example Mobile Robot with L515

```bash
# Test the integration example
ros2 launch realsense2_description example_robot_with_l515.launch.py
```

**What to verify:**
1. ✅ Mobile robot spawns in Gazebo with L515 mounted on top
2. ✅ Camera topics are publishing under `/robot_camera/` namespace
3. ✅ Robot can be controlled (optional - send velocity commands)

**Check the camera topics:**
```bash
ros2 topic list | grep robot_camera
```

**Expected:**
```
/robot_camera/color/camera_info
/robot_camera/color/image_raw
/robot_camera/depth/camera_info
/robot_camera/depth/image_rect_raw
/robot_camera/depth/color/points
/robot_camera/infra/camera_info
/robot_camera/infra/image_raw
```

### 10. Test 6: Point Cloud Verification

```bash
# Check if point cloud is being published
source ~/test_l515_ws/install/setup.bash
ros2 topic echo /camera/depth/color/points --once
```

**What to verify:**
- `height:` and `width:` are non-zero
- `fields:` should include x, y, z, rgb
- `data:` should contain actual point cloud data

### 11. Test 7: Plugin Loading

```bash
# Check Gazebo plugin is loaded correctly
# In the terminal running Gazebo, look for:
```

**Expected log messages:**
```
[INFO] [1234567890.123456] [realsense_camera]: Realsense Gazebo ROS2 plugin loading.
```

**No errors like:**
```
[ERROR] plugin libXXX.so not found
[ERROR] Failed to load plugin
```

### 12. Performance Test

Monitor system performance while running:

```bash
# In a new terminal
top
# or
htop

# Check CPU usage - should be reasonable
# Plugin should not cause excessive CPU usage
```

## Common Issues and Solutions

### Issue 1: Build Fails with Missing Dependencies

**Solution:**
```bash
# Install missing ROS2 packages
sudo apt install ros-${ROS_DISTRO}-<missing-package>

# Or use rosdep
rosdep install --from-paths src --ignore-src -r -y
```

### Issue 2: "Package not found" Error

**Solution:**
```bash
# Make sure workspace is sourced
source ~/test_l515_ws/install/setup.bash

# Verify build succeeded
colcon list
```

### Issue 3: No Image Data Published

**Possible causes:**
- Gazebo not running
- Plugin not loaded correctly
- Topics have different namespace

**Solution:**
```bash
# Check if Gazebo is running
ps aux | grep gazebo

# Check actual topics being published
ros2 topic list

# Check for errors in Gazebo terminal
```

### Issue 4: Point Cloud Not Publishing

**This is normal!** The point cloud only publishes when there are subscribers.

**Solution:**
```bash
# Start a subscriber (like RViz) then check again
ros2 topic hz /camera/depth/color/points
```

### Issue 5: Camera Appears Black in RViz

**Possible causes:**
- Camera facing wrong direction
- No objects in view
- Image encoding mismatch

**Solution:**
```bash
# Check image encoding
ros2 topic echo /camera/color/image_raw --field encoding

# Add objects to Gazebo world to give camera something to see
```

## Automated Test Script

Save this as `test_l515.sh`:

```bash
#!/bin/bash

echo "=== ROS2 L515 Package Test ==="
echo ""

# Test 1: Check if packages exist
echo "Test 1: Checking packages..."
if ros2 pkg list | grep -q "realsense2_description" && ros2 pkg list | grep -q "realsense_gazebo_plugin"; then
    echo "✅ Packages found"
else
    echo "❌ Packages not found"
    exit 1
fi

# Test 2: Check if launch files exist
echo "Test 2: Checking launch files..."
if ros2 pkg prefix realsense2_description > /dev/null 2>&1; then
    PKG_PATH=$(ros2 pkg prefix realsense2_description)
    if [ -f "$PKG_PATH/share/realsense2_description/launch/gazebo.launch.py" ]; then
        echo "✅ Launch files found"
    else
        echo "❌ Launch files not found"
        exit 1
    fi
fi

# Test 3: Check if plugin library exists
echo "Test 3: Checking plugin library..."
if ros2 pkg prefix realsense_gazebo_plugin > /dev/null 2>&1; then
    PKG_PATH=$(ros2 pkg prefix realsense_gazebo_plugin)
    if [ -f "$PKG_PATH/lib/librealsense_gazebo_plugin.so" ]; then
        echo "✅ Plugin library found"
    else
        echo "❌ Plugin library not found"
        exit 1
    fi
fi

echo ""
echo "=== All basic tests passed! ==="
echo "Run manual tests to verify functionality."
```

Run it:
```bash
chmod +x test_l515.sh
./test_l515.sh
```

## Final Verification Checklist

Use this checklist to confirm everything works:

- [ ] Package builds without errors
- [ ] Packages are found in ROS2 environment
- [ ] Gazebo launches with L515 camera
- [ ] Camera model appears correctly in Gazebo
- [ ] Color image topic publishes at ~30 Hz
- [ ] Depth image topic publishes at ~30 Hz
- [ ] Camera info topics publish
- [ ] Point cloud publishes when subscribed
- [ ] Images visible in RViz
- [ ] Point cloud visible in RViz
- [ ] TF tree is correct
- [ ] Example robot launches successfully
- [ ] Integration example works
- [ ] No error messages in terminal

## Success Criteria

Your ROS2 port is working correctly if:

1. ✅ **Build Success**: `colcon build` completes without errors
2. ✅ **Launch Success**: All launch files start without errors
3. ✅ **Data Publishing**: All sensor topics publish data at expected rates
4. ✅ **Visualization Works**: Images and point clouds visible in RViz
5. ✅ **Integration Works**: Example robot with L515 functions correctly
6. ✅ **No Runtime Errors**: No errors in Gazebo or ROS2 logs

## Reporting Issues

If tests fail, collect this information:

```bash
# ROS2 version
printenv ROS_DISTRO

# Gazebo version
gazebo --version

# Build log
colcon build --event-handlers console_direct+ 2>&1 | tee build_log.txt

# Runtime log
ros2 launch realsense2_description gazebo.launch.py 2>&1 | tee runtime_log.txt

# Topic list
ros2 topic list > topics.txt

# Node list
ros2 node list > nodes.txt
```

Attach these files when reporting issues.

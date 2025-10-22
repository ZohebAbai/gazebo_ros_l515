# L515 Camera Integration Guide

This guide explains how to add the Intel RealSense L515 camera sensor to your existing robot model in Gazebo.

## Quick Start

The L515 sensor is provided as a reusable **xacro macro** that can be easily included in any robot URDF.

### Basic Integration Steps

1. **Include the L515 xacro file** in your robot's URDF/xacro
2. **Call the sensor macro** with appropriate parameters
3. **Attach it to any link** in your robot

## Detailed Example

### Step 1: Include the L515 Xacro

Add this line to your robot's main URDF/xacro file:

```xml
<xacro:include filename="$(find realsense2_description)/urdf/_l515.urdf.xacro" />
```

### Step 2: Use the Sensor Macro

Add the sensor to your robot by calling the `sensor_l515` macro:

```xml
<xacro:sensor_l515
    name="camera"
    topics_ns="camera"
    parent="your_robot_link"
    use_nominal_extrinsics="true"
    publish_pointcloud="true">
    <origin xyz="0.1 0 0.2" rpy="0 0 0" />
</xacro:sensor_l515>
```

### Complete Example: Robot with L515 Camera

Here's a complete example showing how to add the L515 to a simple mobile robot:

```xml
<?xml version="1.0"?>
<robot name="my_robot_with_l515" xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- Include the L515 sensor macro -->
  <xacro:include filename="$(find realsense2_description)/urdf/_l515.urdf.xacro" />

  <!-- Your robot's base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Add the L515 camera -->
  <!-- Mounted on top of the robot, 0.1m forward, 0.15m up -->
  <xacro:sensor_l515
      name="front_camera"
      topics_ns="front_camera"
      parent="base_link"
      use_nominal_extrinsics="true"
      publish_pointcloud="true"
      add_plug="false"
      use_mesh="true">
      <origin xyz="0.25 0 0.15" rpy="0 0 0" />
  </xacro:sensor_l515>

</robot>
```

## Macro Parameters

The `sensor_l515` macro accepts the following parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `name` | string | `camera` | Name prefix for all camera links and joints |
| `topics_ns` | string | `camera` | ROS topic namespace for camera data |
| `parent` | string | **required** | Parent link to attach the camera to |
| `use_nominal_extrinsics` | boolean | `false` | Use nominal extrinsics (set to `true` for simulation) |
| `publish_pointcloud` | boolean | `false` | Enable point cloud publication |
| `add_plug` | boolean | `false` | Add USB plug visual (cosmetic) |
| `use_mesh` | boolean | `true` | Use actual L515 mesh vs simple cylinder |

### Origin Block

The `<origin>` block inside the macro call defines where the camera is mounted relative to the parent link:

```xml
<origin xyz="x y z" rpy="roll pitch yaw" />
```

- `xyz`: Position offset in meters (forward, left, up)
- `rpy`: Orientation in radians (roll, pitch, yaw)

## ROS Topics Published

When integrated, the L515 sensor will publish to the following topics (assuming `topics_ns="camera"`):

**For ROS 1:**
- `/camera/color/image_raw` - RGB color image
- `/camera/color/camera_info` - Color camera calibration
- `/camera/depth/image_rect_raw` - Depth image
- `/camera/depth/camera_info` - Depth camera calibration
- `/camera/infra/image_raw` - Infrared image
- `/camera/infra/camera_info` - Infrared camera calibration
- `/camera/depth/color/points` - Point cloud (if enabled)

**For ROS 2:**
Same topic names, accessed via `ros2 topic list`

## Common Integration Examples

### Example 1: Front-Facing Camera on Mobile Robot

```xml
<xacro:sensor_l515
    name="front_camera"
    topics_ns="front_camera"
    parent="base_link"
    use_nominal_extrinsics="true"
    publish_pointcloud="true">
    <origin xyz="0.2 0 0.1" rpy="0 0 0" />
</xacro:sensor_l515>
```

### Example 2: Tilted Downward Camera (for ground detection)

```xml
<xacro:sensor_l515
    name="ground_camera"
    topics_ns="ground_camera"
    parent="base_link"
    use_nominal_extrinsics="true"
    publish_pointcloud="false">
    <!-- Tilted 30 degrees downward (pitch = -0.524 rad) -->
    <origin xyz="0.15 0 0.1" rpy="0 -0.524 0" />
</xacro:sensor_l515>
```

### Example 3: Multiple Cameras on Same Robot

```xml
<!-- Front camera -->
<xacro:sensor_l515
    name="front_camera"
    topics_ns="front_camera"
    parent="base_link"
    use_nominal_extrinsics="true"
    publish_pointcloud="true">
    <origin xyz="0.2 0 0.1" rpy="0 0 0" />
</xacro:sensor_l515>

<!-- Rear camera -->
<xacro:sensor_l515
    name="rear_camera"
    topics_ns="rear_camera"
    parent="base_link"
    use_nominal_extrinsics="true"
    publish_pointcloud="false">
    <origin xyz="-0.2 0 0.1" rpy="0 0 3.14159" />
</xacro:sensor_l515>
```

### Example 4: Camera on Robot Arm End-Effector

```xml
<xacro:sensor_l515
    name="gripper_camera"
    topics_ns="gripper_camera"
    parent="end_effector_link"
    use_nominal_extrinsics="true"
    publish_pointcloud="true">
    <origin xyz="0.05 0 0" rpy="0 0 0" />
</xacro:sensor_l515>
```

## Important Notes

### For Simulation (Gazebo)
- **Always set** `use_nominal_extrinsics="true"` for Gazebo simulation
- The sensor automatically includes the Gazebo plugin configuration
- Point cloud generation can be resource-intensive; enable only if needed

### Performance Considerations
- Point cloud publishing (`publish_pointcloud="true"`) is computationally expensive
- For better performance, disable point clouds if you only need RGB or depth images
- Multiple cameras will multiply the computational load

### Frame Names
The sensor creates multiple frames:
- `{name}_link` - Main camera physical link
- `{name}_depth_optical_frame` - Depth camera optical frame
- `{name}_color_optical_frame` - Color camera optical frame
- `{name}_infra_optical_frame` - Infrared camera optical frame

These frames can be used in TF tree for transformations.

## Troubleshooting

### Camera not visible in Gazebo
- Check that the parent link exists in your robot
- Verify the origin position doesn't place the camera inside another link
- Make sure `use_mesh="true"` if you want to see the actual camera model

### No topics published
- Ensure you've built the workspace with `colcon build` (ROS 2) or `catkin_make` (ROS 1)
- Check that the Gazebo plugin is loaded: `ros2 topic list` or `rostopic list`
- Verify the `topics_ns` parameter matches your expected namespace

### Point cloud not publishing
- Confirm `publish_pointcloud="true"` is set
- Check subscriber count: the plugin only publishes when there are subscribers (ROS 2)
- Monitor `/camera/depth/color/points` topic

## Advanced Customization

If you need to customize sensor parameters (resolution, FOV, update rates), you can modify the `_l515.gazebo.xacro` file or create your own gazebo macro based on it.

### Customizable Parameters in Gazebo Macro

The Gazebo sensor parameters are defined in `_l515.gazebo.xacro`:
- Image resolution (width/height)
- Horizontal FOV
- Update rates
- Noise characteristics
- Depth range (min/max)

## Testing Your Integration

After adding the L515 to your robot:

1. **Build your workspace:**
   ```bash
   # ROS 2
   colcon build --symlink-install
   source install/setup.bash

   # ROS 1
   catkin_make
   source devel/setup.bash
   ```

2. **Launch Gazebo with your robot:**
   ```bash
   # ROS 2
   ros2 launch your_robot_description your_robot_gazebo.launch.py

   # ROS 1
   roslaunch your_robot_description your_robot_gazebo.launch
   ```

3. **Verify topics:**
   ```bash
   # ROS 2
   ros2 topic list
   ros2 topic echo /camera/color/image_raw

   # ROS 1
   rostopic list
   rostopic echo /camera/color/image_raw
   ```

4. **Visualize in RViz:**
   - Add an Image display
   - Subscribe to `/camera/color/image_raw`
   - Add PointCloud2 display for `/camera/depth/color/points`

## Example Files

See `test_l515_camera.urdf.xacro` in the `realsense2_description/urdf/` folder for a minimal working example.

## Support

For issues or questions:
- Check the repository: https://github.com/ZohebAbai/gazebo_ros_l515
- Open an issue with your URDF and error messages
- Include your ROS version and Gazebo version

## References

- [RealSense L515 Datasheet](https://www.intelrealsense.com/depth-camera-l515/)
- [Xacro Documentation](http://wiki.ros.org/xacro)
- [Gazebo Sensor Plugins](http://gazebosim.org/tutorials?cat=sensors)

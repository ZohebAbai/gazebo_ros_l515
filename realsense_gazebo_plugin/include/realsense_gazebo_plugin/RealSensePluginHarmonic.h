/*
// Copyright (c) 2016 Intel Corporation
// Copyright (c) 2025 - Gazebo Harmonic Port
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#ifndef REALSENSE_PLUGIN_HARMONIC_H_
#define REALSENSE_PLUGIN_HARMONIC_H_

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DepthCamera.hh>
#include <gz/sim/components/Camera.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/CameraSensor.hh>
#include <gz/sensors/DepthCameraSensor.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/DepthCamera.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <memory>
#include <string>
#include <vector>

namespace realsense_gazebo_plugin
{

#define DEPTH_CAMERA_NAME "depth"
#define COLOR_CAMERA_NAME "color"
#define IRED_CAMERA_NAME "ired"

struct CameraParams {
  std::string topic_name;
  std::string camera_info_topic_name;
  std::string optical_frame;
};

/// \brief A Gazebo Harmonic plugin that simulates Real Sense L515 camera streams.
class RealSensePluginHarmonic
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  /// \brief Constructor.
  RealSensePluginHarmonic();

  /// \brief Destructor.
  ~RealSensePluginHarmonic() override;

  // Documentation inherited
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  // Documentation inherited
  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

  // Documentation inherited
  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override;

private:
  /// \brief Initialize ROS2 node and publishers
  void InitializeROS();

  /// \brief Find and store sensor entities
  void FindSensors(gz::sim::EntityComponentManager &_ecm);

  /// \brief Process depth camera data and publish
  void PublishDepthImage(const gz::sim::UpdateInfo &_info);

  /// \brief Process color camera data and publish
  void PublishColorImage(const gz::sim::UpdateInfo &_info);

  /// \brief Process infrared camera data and publish
  void PublishInfraredImage(const gz::sim::UpdateInfo &_info);

  /// \brief Publish point cloud
  void PublishPointCloud(const gz::sim::UpdateInfo &_info);

  /// \brief Create camera info message
  sensor_msgs::msg::CameraInfo CreateCameraInfo(
      const sensor_msgs::msg::Image &image,
      float horizontal_fov);

  /// \brief Model entity
  gz::sim::Entity modelEntity;

  /// \brief Model name
  std::string modelName;

  /// \brief ROS2 node
  rclcpp::Node::SharedPtr rosNode;

  /// \brief Image transport
  std::shared_ptr<image_transport::ImageTransport> imageTransport;

  /// \brief Camera info manager
  std::shared_ptr<camera_info_manager::CameraInfoManager> cameraInfoManager;

  /// \brief Publishers
  image_transport::CameraPublisher colorPub;
  image_transport::CameraPublisher depthPub;
  image_transport::CameraPublisher irPub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPub;

  /// \brief Camera parameters
  std::map<std::string, CameraParams> cameraParamsMap;

  /// \brief Sensor entities
  gz::sim::Entity depthCameraEntity;
  gz::sim::Entity colorCameraEntity;
  gz::sim::Entity iredCameraEntity;

  /// \brief Sensor pointers
  gz::sensors::DepthCameraSensor *depthSensor{nullptr};
  gz::sensors::CameraSensor *colorSensor{nullptr};
  gz::sensors::CameraSensor *iredSensor{nullptr};

  /// \brief Update rates
  double colorUpdateRate{30.0};
  double infraredUpdateRate{30.0};
  double depthUpdateRate{30.0};

  /// \brief Range parameters
  float rangeMinDepth{0.1f};
  float rangeMaxDepth{10.0f};

  /// \brief Point cloud parameters
  bool pointCloud{false};
  std::string pointCloudTopic;
  double pointCloudCutOff{0.1};
  double pointCloudCutOffMax{9.0};

  /// \brief Depth scale (meters per unit)
  static constexpr double DEPTH_SCALE_M = 0.00025;

  /// \brief Depth map storage
  std::vector<uint16_t> depthMap;

  /// \brief Message storage
  sensor_msgs::msg::Image colorMsg;
  sensor_msgs::msg::Image depthMsg;
  sensor_msgs::msg::Image irMsg;
  sensor_msgs::msg::PointCloud2 pointCloudMsg;

  /// \brief Prefix for sensor names
  std::string prefix;

  /// \brief Last update times for rate limiting
  std::chrono::steady_clock::time_point lastColorUpdate;
  std::chrono::steady_clock::time_point lastDepthUpdate;
  std::chrono::steady_clock::time_point lastIrUpdate;

  /// \brief Sensor factory
  gz::sensors::SensorFactory sensorFactory;

  /// \brief Flag to check if initialized
  bool initialized{false};
};

}  // namespace realsense_gazebo_plugin

#endif  // REALSENSE_PLUGIN_HARMONIC_H_

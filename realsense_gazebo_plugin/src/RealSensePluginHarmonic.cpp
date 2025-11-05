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

#include "realsense_gazebo_plugin/RealSensePluginHarmonic.h"
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/Util.hh>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace realsense_gazebo_plugin;

// Register the plugin
GZ_ADD_PLUGIN(
    RealSensePluginHarmonic,
    gz::sim::System,
    RealSensePluginHarmonic::ISystemConfigure,
    RealSensePluginHarmonic::ISystemPreUpdate,
    RealSensePluginHarmonic::ISystemPostUpdate)

RealSensePluginHarmonic::RealSensePluginHarmonic()
{
  this->pointCloudCutOffMax = 9.0;
}

RealSensePluginHarmonic::~RealSensePluginHarmonic()
{
}

void RealSensePluginHarmonic::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/)
{
  this->modelEntity = _entity;
  auto model = gz::sim::Model(_entity);
  this->modelName = model.Name(_ecm);

  gzmsg << "RealSensePluginHarmonic: Loading plugin for model "
        << this->modelName << std::endl;

  // Initialize camera parameters map
  cameraParamsMap[COLOR_CAMERA_NAME] = CameraParams();
  cameraParamsMap[DEPTH_CAMERA_NAME] = CameraParams();
  cameraParamsMap[IRED_CAMERA_NAME] = CameraParams();

  // Parse SDF parameters
  if (_sdf->HasElement("depthUpdateRate"))
    this->depthUpdateRate = _sdf->Get<double>("depthUpdateRate");

  if (_sdf->HasElement("colorUpdateRate"))
    this->colorUpdateRate = _sdf->Get<double>("colorUpdateRate");

  if (_sdf->HasElement("infraredUpdateRate"))
    this->infraredUpdateRate = _sdf->Get<double>("infraredUpdateRate");

  if (_sdf->HasElement("depthTopicName"))
    cameraParamsMap[DEPTH_CAMERA_NAME].topic_name =
        _sdf->Get<std::string>("depthTopicName");

  if (_sdf->HasElement("depthCameraInfoTopicName"))
    cameraParamsMap[DEPTH_CAMERA_NAME].camera_info_topic_name =
        _sdf->Get<std::string>("depthCameraInfoTopicName");

  if (_sdf->HasElement("colorTopicName"))
    cameraParamsMap[COLOR_CAMERA_NAME].topic_name =
        _sdf->Get<std::string>("colorTopicName");

  if (_sdf->HasElement("colorCameraInfoTopicName"))
    cameraParamsMap[COLOR_CAMERA_NAME].camera_info_topic_name =
        _sdf->Get<std::string>("colorCameraInfoTopicName");

  if (_sdf->HasElement("infraredTopicName"))
    cameraParamsMap[IRED_CAMERA_NAME].topic_name =
        _sdf->Get<std::string>("infraredTopicName");

  if (_sdf->HasElement("infraredCameraInfoTopicName"))
    cameraParamsMap[IRED_CAMERA_NAME].camera_info_topic_name =
        _sdf->Get<std::string>("infraredCameraInfoTopicName");

  if (_sdf->HasElement("colorOpticalframeName"))
    cameraParamsMap[COLOR_CAMERA_NAME].optical_frame =
        _sdf->Get<std::string>("colorOpticalframeName");

  if (_sdf->HasElement("depthOpticalframeName"))
    cameraParamsMap[DEPTH_CAMERA_NAME].optical_frame =
        _sdf->Get<std::string>("depthOpticalframeName");

  if (_sdf->HasElement("infraredOpticalframeName"))
    cameraParamsMap[IRED_CAMERA_NAME].optical_frame =
        _sdf->Get<std::string>("infraredOpticalframeName");

  if (_sdf->HasElement("rangeMinDepth"))
    this->rangeMinDepth = _sdf->Get<float>("rangeMinDepth");

  if (_sdf->HasElement("rangeMaxDepth"))
    this->rangeMaxDepth = _sdf->Get<float>("rangeMaxDepth");

  if (_sdf->HasElement("pointCloud"))
    this->pointCloud = _sdf->Get<bool>("pointCloud");

  if (_sdf->HasElement("pointCloudTopicName"))
    this->pointCloudTopic = _sdf->Get<std::string>("pointCloudTopicName");

  if (_sdf->HasElement("pointCloudCutoff"))
    this->pointCloudCutOff = _sdf->Get<double>("pointCloudCutoff");

  if (_sdf->HasElement("pointCloudCutoffMax"))
    this->pointCloudCutOffMax = _sdf->Get<double>("pointCloudCutoffMax");

  if (_sdf->HasElement("prefix"))
    this->prefix = _sdf->Get<std::string>("prefix");

  // Initialize ROS2
  InitializeROS();

  gzmsg << "RealSensePluginHarmonic: Configuration complete" << std::endl;
}

void RealSensePluginHarmonic::InitializeROS()
{
  // Initialize ROS2 node
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }

  this->rosNode = rclcpp::Node::make_shared(this->modelName + "_realsense");

  // Initialize image transport
  this->imageTransport = std::make_shared<image_transport::ImageTransport>(this->rosNode);

  // Initialize camera info manager
  this->cameraInfoManager = std::make_shared<camera_info_manager::CameraInfoManager>(
      this->rosNode.get(), this->modelName);

  // Create publishers
  this->colorPub = this->imageTransport->advertiseCamera(
      cameraParamsMap[COLOR_CAMERA_NAME].topic_name, 1);

  this->depthPub = this->imageTransport->advertiseCamera(
      cameraParamsMap[DEPTH_CAMERA_NAME].topic_name, 1);

  this->irPub = this->imageTransport->advertiseCamera(
      cameraParamsMap[IRED_CAMERA_NAME].topic_name, 1);

  if (this->pointCloud)
  {
    this->pointCloudPub = this->rosNode->create_publisher<sensor_msgs::msg::PointCloud2>(
        this->pointCloudTopic, 10);
  }

  gzmsg << "RealSensePluginHarmonic: ROS2 node initialized" << std::endl;
}

void RealSensePluginHarmonic::PreUpdate(
    const gz::sim::UpdateInfo &/*_info*/,
    gz::sim::EntityComponentManager &_ecm)
{
  // Find sensors on first run
  if (!this->initialized)
  {
    FindSensors(_ecm);
    this->initialized = true;
  }
}

void RealSensePluginHarmonic::FindSensors(gz::sim::EntityComponentManager &_ecm)
{
  // Iterate through all entities to find our sensors
  _ecm.Each<gz::sim::components::Sensor, gz::sim::components::Name>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::Sensor */*_sensor*/,
        const gz::sim::components::Name *_name) -> bool
    {
      std::string sensorName = _name->Data();

      // Check if this sensor belongs to our model
      if (sensorName.find(this->prefix + DEPTH_CAMERA_NAME) != std::string::npos)
      {
        this->depthCameraEntity = _entity;
        gzmsg << "Found depth camera: " << sensorName << std::endl;
      }
      else if (sensorName.find(this->prefix + COLOR_CAMERA_NAME) != std::string::npos)
      {
        this->colorCameraEntity = _entity;
        gzmsg << "Found color camera: " << sensorName << std::endl;
      }
      else if (sensorName.find(this->prefix + IRED_CAMERA_NAME) != std::string::npos)
      {
        this->iredCameraEntity = _entity;
        gzmsg << "Found IR camera: " << sensorName << std::endl;
      }

      return true;
    });

  gzmsg << "RealSensePluginHarmonic: Sensor discovery complete" << std::endl;
}

void RealSensePluginHarmonic::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &/*_ecm*/)
{
  // Only update if simulation is not paused
  if (_info.paused)
    return;

  // Spin ROS2
  rclcpp::spin_some(this->rosNode);

  // Note: In Gazebo Harmonic, we need to access sensor data through the
  // rendering API or through topics. This implementation uses a simplified
  // approach where we'll rely on gz-sensors to handle the actual image
  // capture and we bridge the data to ROS2.

  // For now, this is a placeholder. A full implementation would require:
  // 1. Accessing gz::sensors::SensorManager
  // 2. Getting sensor data from each camera
  // 3. Converting and publishing to ROS2
  // This typically requires additional setup in the Configure phase
}

sensor_msgs::msg::CameraInfo RealSensePluginHarmonic::CreateCameraInfo(
    const sensor_msgs::msg::Image &image,
    float horizontal_fov)
{
  sensor_msgs::msg::CameraInfo info_msg;

  info_msg.header = image.header;
  info_msg.distortion_model = "plumb_bob";
  info_msg.height = image.height;
  info_msg.width = image.width;

  float focal = 0.5 * image.width / tan(0.5 * horizontal_fov);

  info_msg.k[0] = focal;
  info_msg.k[4] = focal;
  info_msg.k[2] = info_msg.width * 0.5;
  info_msg.k[5] = info_msg.height * 0.5;
  info_msg.k[8] = 1.;

  info_msg.p[0] = info_msg.k[0];
  info_msg.p[5] = info_msg.k[4];
  info_msg.p[2] = info_msg.k[2];
  info_msg.p[6] = info_msg.k[5];
  info_msg.p[10] = info_msg.k[8];

  return info_msg;
}

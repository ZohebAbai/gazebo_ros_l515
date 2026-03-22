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

#include "realsense_gazebo_plugin/RealSensePluginHarmonic.h"

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>

GZ_ADD_PLUGIN(
    realsense_gazebo_plugin::RealSensePluginHarmonic,
    gz::sim::System,
    realsense_gazebo_plugin::RealSensePluginHarmonic::ISystemConfigure,
    realsense_gazebo_plugin::RealSensePluginHarmonic::ISystemPreUpdate,
    realsense_gazebo_plugin::RealSensePluginHarmonic::ISystemPostUpdate)

namespace realsense_gazebo_plugin
{

RealSensePluginHarmonic::RealSensePluginHarmonic() {}
RealSensePluginHarmonic::~RealSensePluginHarmonic() {}

void RealSensePluginHarmonic::Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
  this->modelEntity = _entity;
  gz::sim::Model model(_entity);
  this->modelName = model.Name(_ecm);

  gzmsg << "RealSensePluginHarmonic: Loading plugin for model "
        << this->modelName << std::endl;

  cameraParamsMap[COLOR_CAMERA_NAME] = CameraParams();
  cameraParamsMap[DEPTH_CAMERA_NAME] = CameraParams();
  cameraParamsMap[IRED_CAMERA_NAME]  = CameraParams();

  if (_sdf->HasElement("depthUpdateRate")) {
    this->depthUpdateRate = _sdf->Get<double>("depthUpdateRate");
  }
  if (_sdf->HasElement("colorUpdateRate")) {
    this->colorUpdateRate = _sdf->Get<double>("colorUpdateRate");
  }
  if (_sdf->HasElement("infraredUpdateRate")) {
    this->infraredUpdateRate = _sdf->Get<double>("infraredUpdateRate");
  }
  if (_sdf->HasElement("depthTopicName")) {
    cameraParamsMap[DEPTH_CAMERA_NAME].topic_name =
        _sdf->Get<std::string>("depthTopicName");
  }
  if (_sdf->HasElement("depthCameraInfoTopicName")) {
    cameraParamsMap[DEPTH_CAMERA_NAME].camera_info_topic_name =
        _sdf->Get<std::string>("depthCameraInfoTopicName");
  }
  if (_sdf->HasElement("colorTopicName")) {
    cameraParamsMap[COLOR_CAMERA_NAME].topic_name =
        _sdf->Get<std::string>("colorTopicName");
  }
  if (_sdf->HasElement("colorCameraInfoTopicName")) {
    cameraParamsMap[COLOR_CAMERA_NAME].camera_info_topic_name =
        _sdf->Get<std::string>("colorCameraInfoTopicName");
  }
  if (_sdf->HasElement("infraredTopicName")) {
    cameraParamsMap[IRED_CAMERA_NAME].topic_name =
        _sdf->Get<std::string>("infraredTopicName");
  }
  if (_sdf->HasElement("infraredCameraInfoTopicName")) {
    cameraParamsMap[IRED_CAMERA_NAME].camera_info_topic_name =
        _sdf->Get<std::string>("infraredCameraInfoTopicName");
  }
  if (_sdf->HasElement("colorOpticalframeName")) {
    cameraParamsMap[COLOR_CAMERA_NAME].optical_frame =
        _sdf->Get<std::string>("colorOpticalframeName");
  }
  if (_sdf->HasElement("depthOpticalframeName")) {
    cameraParamsMap[DEPTH_CAMERA_NAME].optical_frame =
        _sdf->Get<std::string>("depthOpticalframeName");
  }
  if (_sdf->HasElement("infraredOpticalframeName")) {
    cameraParamsMap[IRED_CAMERA_NAME].optical_frame =
        _sdf->Get<std::string>("infraredOpticalframeName");
  }
  if (_sdf->HasElement("rangeMinDepth")) {
    this->rangeMinDepth = _sdf->Get<float>("rangeMinDepth");
  }
  if (_sdf->HasElement("rangeMaxDepth")) {
    this->rangeMaxDepth = _sdf->Get<float>("rangeMaxDepth");
  }
  if (_sdf->HasElement("pointCloud")) {
    this->pointCloud = _sdf->Get<bool>("pointCloud");
  }
  if (_sdf->HasElement("pointCloudTopicName")) {
    this->pointCloudTopic = _sdf->Get<std::string>("pointCloudTopicName");
  }
  if (_sdf->HasElement("pointCloudCutoff")) {
    this->pointCloudCutOff = _sdf->Get<double>("pointCloudCutoff");
  }
  if (_sdf->HasElement("pointCloudCutoffMax")) {
    this->pointCloudCutOffMax = _sdf->Get<double>("pointCloudCutoffMax");
  }
  if (_sdf->HasElement("prefix")) {
    this->prefix = _sdf->Get<std::string>("prefix");
  }

  gzmsg << "RealSensePluginHarmonic: Bridge expects\n"
        << "  color on:  " << cameraParamsMap[COLOR_CAMERA_NAME].topic_name << "\n"
        << "  depth on:  " << cameraParamsMap[DEPTH_CAMERA_NAME].topic_name << "\n"
        << "  IR on:     " << cameraParamsMap[IRED_CAMERA_NAME].topic_name  << "\n"
        << "  Data bridged via ros_gz_bridge/ros_gz_image." << std::endl;
}

void RealSensePluginHarmonic::PreUpdate(
    const gz::sim::UpdateInfo & /*_info*/,
    gz::sim::EntityComponentManager & _ecm)
{
  if (!this->initialized) {
    FindSensors(_ecm);
    this->initialized = true;
  }
}

void RealSensePluginHarmonic::FindSensors(gz::sim::EntityComponentManager & _ecm)
{
  _ecm.Each<gz::sim::components::Sensor, gz::sim::components::Name>(
    [&](const gz::sim::Entity & _entity,
        const gz::sim::components::Sensor * /*_sensor*/,
        const gz::sim::components::Name * _name) -> bool
    {
      const std::string & sensorName = _name->Data();

      if (sensorName.find(this->prefix + DEPTH_CAMERA_NAME) != std::string::npos) {
        this->depthCameraEntity = _entity;
        gzmsg << "RealSensePluginHarmonic: Found depth sensor: " << sensorName << std::endl;
      } else if (sensorName.find(this->prefix + COLOR_CAMERA_NAME) != std::string::npos) {
        this->colorCameraEntity = _entity;
        gzmsg << "RealSensePluginHarmonic: Found color sensor: " << sensorName << std::endl;
      } else if (sensorName.find(this->prefix + IRED_CAMERA_NAME) != std::string::npos) {
        this->iredCameraEntity = _entity;
        gzmsg << "RealSensePluginHarmonic: Found IR sensor: " << sensorName << std::endl;
      }

      return true;
    });

  if (this->depthCameraEntity == gz::sim::kNullEntity) {
    gzwarn << "RealSensePluginHarmonic: depth sensor not found. "
              "Check <prefix> matches URDF sensor name." << std::endl;
  }
  if (this->colorCameraEntity == gz::sim::kNullEntity) {
    gzwarn << "RealSensePluginHarmonic: color sensor not found." << std::endl;
  }
  if (this->iredCameraEntity == gz::sim::kNullEntity) {
    gzwarn << "RealSensePluginHarmonic: IR sensor not found." << std::endl;
  }

  this->sensorsFound =
      (this->depthCameraEntity != gz::sim::kNullEntity) &&
      (this->colorCameraEntity != gz::sim::kNullEntity) &&
      (this->iredCameraEntity  != gz::sim::kNullEntity);

  if (this->sensorsFound) {
    gzmsg << "RealSensePluginHarmonic: All sensors found. "
             "ros_gz_bridge nodes handle data bridging." << std::endl;
  }
}

void RealSensePluginHarmonic::PostUpdate(
    const gz::sim::UpdateInfo & _info,
    const gz::sim::EntityComponentManager & /*_ecm*/)
{
  if (_info.paused) {
    return;
  }
  // Sensor data is bridged to ROS2 by ros_gz_bridge / ros_gz_image nodes
  // launched alongside this simulation. No action required here.
}

}  // namespace realsense_gazebo_plugin

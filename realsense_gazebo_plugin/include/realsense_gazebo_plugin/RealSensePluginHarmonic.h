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

#ifndef REALSENSE_GAZEBO_PLUGIN__REALSENSEPLUGINHARMONIC_H_
#define REALSENSE_GAZEBO_PLUGIN__REALSENSEPLUGINHARMONIC_H_

#include <map>
#include <memory>
#include <string>

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DepthCamera.hh>
#include <gz/sim/components/Camera.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Sensor.hh>

namespace realsense_gazebo_plugin
{

#define DEPTH_CAMERA_NAME "depth"
#define COLOR_CAMERA_NAME "color"
#define IRED_CAMERA_NAME "ired"

/// \brief Parameters parsed from SDF for each camera stream.
/// These document what topic names the ros_gz_bridge nodes must subscribe to.
struct CameraParams
{
  std::string topic_name;
  std::string camera_info_topic_name;
  std::string optical_frame;
};

/// \brief Gazebo Harmonic system plugin for the RealSense L515 camera.
///
/// This plugin handles SDF parameter parsing, sensor entity discovery, and
/// lifecycle logging. Actual sensor data is bridged to ROS2 by ros_gz_bridge
/// and ros_gz_image nodes launched alongside this simulation.
class RealSensePluginHarmonic
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  RealSensePluginHarmonic();
  ~RealSensePluginHarmonic() override;

  void Configure(
      const gz::sim::Entity & _entity,
      const std::shared_ptr<const sdf::Element> & _sdf,
      gz::sim::EntityComponentManager & _ecm,
      gz::sim::EventManager & _eventMgr) override;

  void PreUpdate(
      const gz::sim::UpdateInfo & _info,
      gz::sim::EntityComponentManager & _ecm) override;

  void PostUpdate(
      const gz::sim::UpdateInfo & _info,
      const gz::sim::EntityComponentManager & _ecm) override;

private:
  /// \brief Iterate ECS to locate all three sensor entities by name.
  void FindSensors(gz::sim::EntityComponentManager & _ecm);

  gz::sim::Entity modelEntity;
  std::string modelName;

  /// \brief Camera params parsed from SDF (documents expected bridge topics).
  std::map<std::string, CameraParams> cameraParamsMap;

  /// \brief Sensor entities discovered in FindSensors().
  gz::sim::Entity depthCameraEntity{gz::sim::kNullEntity};
  gz::sim::Entity colorCameraEntity{gz::sim::kNullEntity};
  gz::sim::Entity iredCameraEntity{gz::sim::kNullEntity};

  /// \brief Expected update rates (parsed from SDF for logging only).
  double colorUpdateRate{30.0};
  double infraredUpdateRate{90.0};
  double depthUpdateRate{90.0};

  float rangeMinDepth{0.2f};
  float rangeMaxDepth{10.0f};

  bool pointCloud{false};
  std::string pointCloudTopic;
  double pointCloudCutOff{0.25};
  double pointCloudCutOffMax{9.0};

  /// \brief Prefix used to match sensor entity names.
  std::string prefix;

  /// \brief True after FindSensors() runs once.
  bool initialized{false};

  /// \brief True if all three sensors were found during discovery.
  bool sensorsFound{false};
};

}  // namespace realsense_gazebo_plugin

#endif  // REALSENSE_GAZEBO_PLUGIN__REALSENSEPLUGINHARMONIC_H_

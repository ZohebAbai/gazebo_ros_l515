cmake_minimum_required(VERSION 3.5)
project(realsense_gazebo_plugin)

# Default to C++17 for Gazebo Harmonic
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(sensor_msgs REQUIRED)

# Find Gazebo Harmonic (new Gazebo for ROS2 Jazzy)
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER 8)

find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER 2)

find_package(gz-sensors8 REQUIRED)
set(GZ_SENSORS_VER 8)

find_package(gz-rendering8 REQUIRED)
set(GZ_RENDERING_VER 8)

find_package(gz-common5 REQUIRED)
set(GZ_COMMON_VER 5)

# Build plugin library
add_library(${PROJECT_NAME} SHARED
    src/RealSensePluginHarmonic.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  image_transport
  camera_info_manager
  sensor_msgs
)

target_link_libraries(${PROJECT_NAME}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
  gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  image_transport
  camera_info_manager
  sensor_msgs
  gz-sim8
  gz-plugin2
  gz-sensors8
  gz-rendering8
  gz-common5
)

ament_package()

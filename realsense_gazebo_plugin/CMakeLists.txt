cmake_minimum_required(VERSION 3.5)
project(realsense_gazebo_plugin)

# Default to C++17
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
find_package(ros_gz_sim REQUIRED)

# Build plugin library
add_library(${PROJECT_NAME} SHARED
    src/RealSensePlugin.cpp
    src/gazebo_ros_realsense.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  image_transport
  camera_info_manager
  sensor_msgs
  ros_gz_sim
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
  ros_gz_sim
)

ament_package()

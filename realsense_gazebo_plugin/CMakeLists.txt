cmake_minimum_required(VERSION 3.5)
project(realsense_gazebo_plugin)

# Default to C++17 for better compatibility
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

# Detect ROS distro and Gazebo version
set(ROS_DISTRO $ENV{ROS_DISTRO})

# Try to determine if we're using Gazebo Classic or new Gazebo (Harmonic/Ignition)
# ROS2 Jazzy and later use new Gazebo, Humble and earlier use Gazebo Classic
if(ROS_DISTRO STREQUAL "jazzy" OR ROS_DISTRO STREQUAL "rolling")
  message(STATUS "Detected ROS2 ${ROS_DISTRO} - Using new Gazebo (Harmonic)")
  set(USE_NEW_GAZEBO TRUE)

  # Find new Gazebo packages
  find_package(gz-sim8 QUIET)
  if(gz-sim8_FOUND)
    set(GZ_SIM_VER 8)
    message(STATUS "Found Gazebo Harmonic (gz-sim8)")
  else()
    find_package(gz-sim7 QUIET)
    if(gz-sim7_FOUND)
      set(GZ_SIM_VER 7)
      message(STATUS "Found Gazebo Garden (gz-sim7)")
    else()
      message(FATAL_ERROR "Could not find gz-sim (version 7 or 8)")
    endif()
  endif()

  find_package(ros_gz_sim REQUIRED)
  set(GAZEBO_DEPENDENCIES ros_gz_sim)

else()
  message(STATUS "Detected ROS2 ${ROS_DISTRO} - Using Gazebo Classic")
  set(USE_NEW_GAZEBO FALSE)

  # Find Gazebo Classic packages
  find_package(gazebo_dev REQUIRED)
  find_package(gazebo_ros REQUIRED)
  set(GAZEBO_DEPENDENCIES gazebo_dev gazebo_ros)

  link_directories(${gazebo_dev_LIBRARY_DIRS})
endif()

# Build plugin library
add_library(${PROJECT_NAME} SHARED
    src/RealSensePlugin.cpp
    src/gazebo_ros_realsense.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Set dependencies based on Gazebo version
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  image_transport
  camera_info_manager
  sensor_msgs
  ${GAZEBO_DEPENDENCIES}
)

# Add compile definitions
if(USE_NEW_GAZEBO)
  target_compile_definitions(${PROJECT_NAME} PRIVATE USE_NEW_GAZEBO)
  target_compile_definitions(${PROJECT_NAME} PRIVATE GZ_SIM_VER=${GZ_SIM_VER})
endif()

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
  ${GAZEBO_DEPENDENCIES}
)

ament_package()

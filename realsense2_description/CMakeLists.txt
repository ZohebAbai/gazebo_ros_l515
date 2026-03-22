cmake_minimum_required(VERSION 3.5)
project(realsense2_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  meshes
  rviz
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_files test/test_launch_files.py TIMEOUT 60)

  install(DIRECTORY test/
    DESTINATION share/${PROJECT_NAME}/test
  )
endif()

ament_package()

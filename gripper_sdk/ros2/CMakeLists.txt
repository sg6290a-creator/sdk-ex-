cmake_minimum_required(VERSION 3.16)
project(gripper_sdk)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../python/gripper_sdk)

install(
  DIRECTORY launch config urdf rviz ../C++
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    gripper_node.py
    gripper_action_server.py
    ../python/gripper_sdk/utils/teleop.py
    ../python/gripper_sdk/utils/calibration.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

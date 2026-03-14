cmake_minimum_required(VERSION 3.16)
project(arm_sdk LANGUAGES CXX)

# ---------------------------------------------------------------------------
# arm_sdk ROS2 어댑터 계층
#
# 구조:
#   C++/ → arm_sdk_native (pure C++: USBCANDevice, RMDProtocol, RobstrideProtocol, ArmDriver)
#   ros2/ → ArmSystemInterface (ArmDriver 를 has-a 멤버로 사용)
#
# C++/폴더는 COLCON_IGNORE 로 독립 빌드 가능, 여기서도 소스를 가져와 컴파일.
# ---------------------------------------------------------------------------

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------
# arm_sdk_native: USBCANDevice + RMDProtocol + RobstrideProtocol + ArmDriver
#                 + IntegratedDriver (pure C++, no ROS)
# ---------------------------------------------------------------------------
add_library(arm_sdk_native STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../C++/src/usbcan_device.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../C++/src/rmd_protocol.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../C++/src/robstride_protocol.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../C++/src/arm_driver.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../C++/src/integrated_driver.cpp
)
set_target_properties(arm_sdk_native PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(arm_sdk_native PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../C++/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(arm_sdk_native PUBLIC Threads::Threads dl)
target_compile_features(arm_sdk_native PUBLIC cxx_std_17)

# ---------------------------------------------------------------------------
# arm_sdk_hw: ArmSystemInterface (thin ros2_control bridge)
# ---------------------------------------------------------------------------
add_library(arm_sdk_system SHARED
  src/arm_system_interface.cpp
)

target_include_directories(arm_sdk_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../C++/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(arm_sdk_system PUBLIC arm_sdk_native)

target_compile_features(arm_sdk_system PUBLIC cxx_std_17)

ament_target_dependencies(arm_sdk_system PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface arm_sdk_plugin.xml)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

# Headers
install(
  DIRECTORY ../C++/include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Libraries
install(
  TARGETS arm_sdk_native arm_sdk_system
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Plugin XML
install(
  FILES arm_sdk_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Launch / config / urdf / rviz / meshes
install(
  DIRECTORY launch config urdf rviz meshes
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Teleop script
install(
  PROGRAMS arm_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  OPTIONAL
)

# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

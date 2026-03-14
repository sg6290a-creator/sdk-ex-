cmake_minimum_required(VERSION 3.14)
project(gripper_sdk_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

option(GRIPPER_SDK_ENABLE_DYNAMIXEL "Build the native gripper C++ backend against Dynamixel SDK." ON)

find_path(DXL_INCLUDE_DIR dynamixel_sdk/dynamixel_sdk.h PATHS /usr/local/include)
find_library(DXL_LIB dxl_x64_cpp PATHS /usr/local/lib)

if(GRIPPER_SDK_ENABLE_DYNAMIXEL AND DXL_INCLUDE_DIR AND DXL_LIB)

  add_library(gripper_sdk_native STATIC
      src/gripper.cpp
      src/motor_group.cpp
      src/impedance.cpp
  )
  target_include_directories(gripper_sdk_native PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${DXL_INCLUDE_DIR}
  )
  target_link_libraries(gripper_sdk_native PUBLIC ${DXL_LIB} pthread)
  target_compile_features(gripper_sdk_native PUBLIC cxx_std_17)

  message(STATUS "[gripper_sdk/C++] gripper_sdk_native built (gripper + motor_group + impedance)")

  # ---------------------------------------------------------------------------
  # Tests (gated by option)
  # ---------------------------------------------------------------------------
  option(GRIPPER_SDK_BUILD_TESTS "Build gripper_sdk test executables." ON)
  if(GRIPPER_SDK_BUILD_TESTS)
    add_executable(test_loop_timing test/test_loop_timing.cpp)
    target_link_libraries(test_loop_timing PRIVATE gripper_sdk_native)

    add_executable(test_stability test/test_stability.cpp)
    target_link_libraries(test_stability PRIVATE gripper_sdk_native)

    add_executable(test_sine test/test_sine.cpp)
    target_link_libraries(test_sine PRIVATE gripper_sdk_native)

    message(STATUS "[gripper_sdk/C++] test_loop_timing, test_stability, test_sine built.")
  endif()

else()
  message(WARNING "Dynamixel C++ SDK not found. Building placeholder target only.")
  add_custom_target(gripper_sdk_native ALL
    COMMENT "Dynamixel SDK unavailable; skipping native gripper C++ backend build."
  )
endif()

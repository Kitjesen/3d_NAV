cmake_minimum_required(VERSION 3.14)

include(cmake/version.cmake)
project(livox_ros_driver2 VERSION ${LIVOX_ROS_DRIVER2_VERSION} LANGUAGES CXX)
message(STATUS "${PROJECT_NAME} version: ${LIVOX_ROS_DRIVER2_VERSION}")

# ── Build options ─────────────────────────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Release or Debug" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wno-unused-parameter)
add_definitions(-DBUILDING_ROS2)

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(std_msgs         REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Livox SDK (installed system-wide via apt or manual build of Livox-SDK2)
find_library(LIVOX_SDK_LIB NAMES livox_lidar_sdk_shared
             PATHS /usr/local/lib REQUIRED)
find_path(LIVOX_SDK_INC NAMES livox_lidar_api.h livox_lidar_def.h REQUIRED)

# ── Custom message types ───────────────────────────────────────────────────────
set(LIVOX_INTERFACES livox_interfaces2)
rosidl_generate_interfaces(${LIVOX_INTERFACES}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES builtin_interfaces std_msgs
  LIBRARY_NAME ${PROJECT_NAME}
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${LIVOX_INTERFACES} "rosidl_typesupport_cpp")

# ── Driver shared library ──────────────────────────────────────────────────────
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/livox_ros_driver2.cpp
  src/lddc.cpp
  src/driver_node.cpp
  src/lds.cpp
  src/lds_lidar.cpp

  src/comm/comm.cpp
  src/comm/ldq.cpp
  src/comm/semaphore.cpp
  src/comm/lidar_imu_data_queue.cpp
  src/comm/cache_index.cpp
  src/comm/pub_handler.cpp

  src/parse_cfg_file/parse_cfg_file.cpp
  src/parse_cfg_file/parse_livox_lidar_cfg.cpp

  src/call_back/lidar_common_callback.cpp
  src/call_back/livox_lidar_callback.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  src
  ${LIVOX_SDK_INC}
)

target_link_libraries(${PROJECT_NAME}
  "${cpp_typesupport_target}"
  ${LIVOX_SDK_LIB}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN     "livox_ros::DriverNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

# ── Install ────────────────────────────────────────────────────────────────────
ament_auto_package(INSTALL_TO_SHARE
  config
  launch_ROS2
)

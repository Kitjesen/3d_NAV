cmake_minimum_required(VERSION 3.8)
project(localizer)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(interface REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_srvs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)

# small_gicp (Koide 2024, MIT) — replaces pcl::IterativeClosestPoint in
# icp_localizer to expose the registration Hessian, iteration count, and
# converged flag. PCL's ICP wrapper exposed only fitness, which forced P3
# multi-frame health gating to single-axis (fitness only). small_gicp is
# header-only friendly via FetchContent and ships a drop-in PCL-style
# RegistrationPCL adapter so the call sites change minimally.
#
# We prefer find_package first (system install path on S100P) and fall
# back to FetchContent when absent so CI / fresh dev machines work.
find_package(small_gicp QUIET)
if(NOT small_gicp_FOUND)
  message(STATUS "small_gicp not installed — fetching v1.0.0")
  include(FetchContent)
  FetchContent_Declare(small_gicp
    GIT_REPOSITORY https://github.com/koide3/small_gicp.git
    GIT_TAG v1.0.0
  )
  FetchContent_MakeAvailable(small_gicp)
endif()


include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  /usr/local/include/bbs3d
)


set(SRC_LIST src/localizers/commons.cpp
             src/localizers/icp_localizer.cpp
             src/localizers/bbs3d_global_localizer.cpp)

add_executable(localizer_node src/localizer_node.cpp ${SRC_LIST})
ament_target_dependencies(localizer_node rclcpp std_msgs sensor_msgs nav_msgs message_filters pcl_conversions tf2_ros geometry_msgs interface std_srvs)
target_link_libraries(localizer_node ${PCL_LIBRARIES} yaml-cpp /usr/local/lib/libcpu_bbs3d.so small_gicp::small_gicp OpenMP::OpenMP_CXX)


install(TARGETS localizer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

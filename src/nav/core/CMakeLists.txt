cmake_minimum_required(VERSION 3.14)
project(nav_core LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MSVC: treat source files as UTF-8 to avoid C4819 warnings.
if(MSVC)
  add_compile_options(/utf-8)
endif()

# Performance flags for all non-MSVC platforms, including aarch64.
# Do not enable relaxed floating-point math globally: validation must preserve NaN/Inf checks.
if(NOT MSVC)
  add_compile_options(-funroll-loops)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT ipo_supported)
  if(ipo_supported)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
  endif()
endif()

# xsimd removed: profile on S100P showed SIMD had no benefit on ARM Cortex-A55
# for our workload sizes. Compiler auto-vectorization handles the hot paths.
include(FetchContent)

# OpenMP is optional and is used for terrain/scoring parallelization.
find_package(OpenMP QUIET)

# Dual mode: ament_cmake for ROS 2, standalone for Windows/CI.
if(DEFINED ENV{AMENT_PREFIX_PATH})
  # ROS 2 mode for S100P / sunrise aarch64.
  find_package(ament_cmake REQUIRED)

  # Header-only library.
  add_library(${PROJECT_NAME} INTERFACE)
  target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} INTERFACE OpenMP::OpenMP_CXX)
    ament_export_dependencies(OpenMP)
  endif()

  ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
  install(DIRECTORY include/ DESTINATION include)
  install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

  # gtest
  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    # Helper: add test with OpenMP.
    macro(nav_core_add_test target src)
      ament_add_gtest(${target} ${src})
      target_include_directories(${target} PRIVATE include)
      if(OpenMP_CXX_FOUND)
        target_link_libraries(${target} OpenMP::OpenMP_CXX)
      endif()
    endmacro()

    nav_core_add_test(test_path_follower_core tests/test_path_follower_core.cpp)
    nav_core_add_test(test_pct_adapter_core tests/test_pct_adapter_core.cpp)
    nav_core_add_test(test_local_planner_core tests/test_local_planner_core.cpp)
    nav_core_add_test(test_param_sensitivity tests/test_param_sensitivity.cpp)
    nav_core_add_test(test_path_edge_cases tests/test_path_edge_cases.cpp)
    nav_core_add_test(test_benchmark tests/test_benchmark.cpp)
    nav_core_add_test(test_validation tests/test_validation.cpp)
  endif()

  ament_package()

else()
  # Standalone mode for Windows, CI, and local development.

  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

  FetchContent_Declare(taskflow
    GIT_REPOSITORY https://github.com/taskflow/taskflow.git
    GIT_TAG        v3.8.0)

  FetchContent_MakeAvailable(googletest taskflow)

  # OpenMP is optional and used for baseline comparison.
  find_package(OpenMP QUIET)

  enable_testing()
  include(GoogleTest)

  add_executable(test_path_follower_core tests/test_path_follower_core.cpp)
  target_include_directories(test_path_follower_core PRIVATE include)
  target_link_libraries(test_path_follower_core GTest::gtest_main)
  gtest_discover_tests(test_path_follower_core)

  add_executable(test_pct_adapter_core tests/test_pct_adapter_core.cpp)
  target_include_directories(test_pct_adapter_core PRIVATE include)
  target_link_libraries(test_pct_adapter_core GTest::gtest_main)
  gtest_discover_tests(test_pct_adapter_core)

  add_executable(test_local_planner_core tests/test_local_planner_core.cpp)
  target_include_directories(test_local_planner_core PRIVATE include)
  target_link_libraries(test_local_planner_core GTest::gtest_main)
  gtest_discover_tests(test_local_planner_core)

  add_executable(test_param_sensitivity tests/test_param_sensitivity.cpp)
  target_include_directories(test_param_sensitivity PRIVATE include)
  target_link_libraries(test_param_sensitivity GTest::gtest_main)
  gtest_discover_tests(test_param_sensitivity)

  add_executable(test_path_edge_cases tests/test_path_edge_cases.cpp)
  target_include_directories(test_path_edge_cases PRIVATE include)
  target_link_libraries(test_path_edge_cases GTest::gtest_main)
  gtest_discover_tests(test_path_edge_cases)

  add_executable(test_benchmark tests/test_benchmark.cpp)
  target_include_directories(test_benchmark PRIVATE include)
  target_link_libraries(test_benchmark GTest::gtest_main Taskflow)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(test_benchmark OpenMP::OpenMP_CXX)
  endif()
  gtest_discover_tests(test_benchmark)

  add_executable(test_validation tests/test_validation.cpp)
  target_include_directories(test_validation PRIVATE include)
  target_link_libraries(test_validation GTest::gtest_main)
  gtest_discover_tests(test_validation)

  option(NAV_CORE_BUILD_PYTHON_BINDINGS "Build nav_core Python bindings" OFF)
  if(NAV_CORE_BUILD_PYTHON_BINDINGS)
    find_package(Python3 COMPONENTS Interpreter Development QUIET)
  endif()
  if(NAV_CORE_BUILD_PYTHON_BINDINGS AND Python3_FOUND AND EXISTS "${Python3_INCLUDE_DIRS}/Python.h")
    # nanobind replaces pybind11 for faster builds, smaller modules, and numpy zero-copy support.
    FetchContent_Declare(nanobind
      GIT_REPOSITORY https://github.com/wjakob/nanobind.git
      GIT_TAG        v2.12.0)
    FetchContent_MakeAvailable(nanobind)
    nanobind_add_module(_nav_core bindings/py_nav_core.cpp)
    target_include_directories(_nav_core PRIVATE include)
  elseif(NAV_CORE_BUILD_PYTHON_BINDINGS)
    message(WARNING "Skipping nav_core Python bindings because Python development headers were not found")
  endif()
endif()

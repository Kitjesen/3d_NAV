# Minimal CMakeLists for building _nav_core.so with nanobind
# Usage on S100P:
#   cp CMakeLists_nanobind_only.cmake CMakeLists.txt
#   cmake -B build_nb -DCMAKE_BUILD_TYPE=Release
#   cmake --build build_nb -j4
#   cp build_nb/_nav_core*.so ~/data/SLAM/navigation/src/
cmake_minimum_required(VERSION 3.14)
project(nav_core_binding LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# nanobind requires find_package(Python ...) not find_package(Python3 ...)
find_package(Python COMPONENTS Interpreter Development REQUIRED)

# nanobind installed via pip: python3 -c "import nanobind; print(nanobind.cmake_dir())"
execute_process(
  COMMAND "${Python_EXECUTABLE}" -c "import nanobind; print(nanobind.cmake_dir())"
  OUTPUT_VARIABLE NB_DIR OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE NB_RET)
if(NOT NB_RET EQUAL 0)
  message(FATAL_ERROR "nanobind not found. Install: pip install nanobind")
endif()
list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
find_package(nanobind CONFIG REQUIRED)

# Build the Python extension module
nanobind_add_module(_nav_core bindings/py_nav_core.cpp)
target_include_directories(_nav_core PRIVATE include)

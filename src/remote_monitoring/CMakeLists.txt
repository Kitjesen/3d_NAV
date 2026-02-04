cmake_minimum_required(VERSION 3.8)
project(remote_monitoring)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Protobuf REQUIRED)

# Find gRPC using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GRPC REQUIRED grpc++)
pkg_check_modules(GRPCPP REQUIRED grpc++>=1.22.0)

# Find grpc_cpp_plugin
find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin REQUIRED)
message(STATUS "Using gRPC: ${GRPC_LIBRARIES}")
message(STATUS "gRPC plugin: ${GRPC_CPP_PLUGIN}")

set(PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/proto)
set(GENERATED_SRC_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${GENERATED_SRC_DIR})

# Proto 文件列表
set(PROTO_FILES
  ${PROTO_DIR}/common.proto
  ${PROTO_DIR}/system.proto
  ${PROTO_DIR}/control.proto
  ${PROTO_DIR}/telemetry.proto
  ${PROTO_DIR}/data.proto
)

# 生成所有 proto 和 grpc 文件
set(PROTO_SRCS)
set(PROTO_HDRS)
set(GRPC_SRCS)
set(GRPC_HDRS)

foreach(PROTO_FILE ${PROTO_FILES})
  get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
  
  list(APPEND PROTO_SRCS ${GENERATED_SRC_DIR}/${PROTO_NAME}.pb.cc)
  list(APPEND PROTO_HDRS ${GENERATED_SRC_DIR}/${PROTO_NAME}.pb.h)
  list(APPEND GRPC_SRCS ${GENERATED_SRC_DIR}/${PROTO_NAME}.grpc.pb.cc)
  list(APPEND GRPC_HDRS ${GENERATED_SRC_DIR}/${PROTO_NAME}.grpc.pb.h)
endforeach()

# Generate proto and grpc files for each proto file
foreach(PROTO_FILE ${PROTO_FILES})
  get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
  
  set(PROTO_OUT ${GENERATED_SRC_DIR}/${PROTO_NAME}.pb.cc)
  set(PROTO_HDR ${GENERATED_SRC_DIR}/${PROTO_NAME}.pb.h)
  set(GRPC_OUT ${GENERATED_SRC_DIR}/${PROTO_NAME}.grpc.pb.cc)
  set(GRPC_HDR ${GENERATED_SRC_DIR}/${PROTO_NAME}.grpc.pb.h)
  
  add_custom_command(
    OUTPUT ${PROTO_OUT} ${PROTO_HDR}
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --cpp_out=${GENERATED_SRC_DIR}
         -I ${PROTO_DIR}
         ${PROTO_FILE}
    DEPENDS ${PROTO_FILE}
    COMMENT "Generating protobuf for ${PROTO_NAME}"
  )
  
  add_custom_command(
    OUTPUT ${GRPC_OUT} ${GRPC_HDR}
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --grpc_out=${GENERATED_SRC_DIR}
         --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
         -I ${PROTO_DIR}
         ${PROTO_FILE}
    DEPENDS ${PROTO_FILE}
    COMMENT "Generating gRPC for ${PROTO_NAME}"
  )
endforeach()

add_executable(grpc_gateway
  src/main.cpp
  src/status_aggregator.cpp
  src/grpc_gateway.cpp
  src/core/lease_manager.cpp
  src/core/event_buffer.cpp
  src/core/safety_gate.cpp
  src/services/system_service.cpp
  src/services/control_service.cpp
  src/services/telemetry_service.cpp
  src/services/data_service.cpp
  ${PROTO_SRCS}
  ${GRPC_SRCS}
)

target_include_directories(grpc_gateway PRIVATE
  ${GENERATED_SRC_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(grpc_gateway PRIVATE
  ${GRPC_INCLUDE_DIRS}
)

target_link_libraries(grpc_gateway
  ${GRPC_LIBRARIES}
  ${GRPCPP_LIBRARIES}
  ${Protobuf_LIBRARIES}
)

ament_target_dependencies(grpc_gateway
  rclcpp nav_msgs sensor_msgs geometry_msgs std_msgs tf2_ros
)

install(TARGETS grpc_gateway
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY proto/
  DESTINATION share/${PROJECT_NAME}/proto
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

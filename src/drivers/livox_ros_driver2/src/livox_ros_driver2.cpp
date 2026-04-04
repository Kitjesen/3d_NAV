// Copyright (c) 2022 Livox. All rights reserved. MIT License.
//
// Trimmed for LingTu / S100P: ROS2 Humble only, single MID-360 lidar.

#include "include/livox_ros_driver2.h"
#include "driver_node.h"
#include "lddc.h"
#include "lds_lidar.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <thread>

namespace livox_ros {

DriverNode::DriverNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("livox_driver_node", options)
{
  RCLCPP_INFO(get_logger(), "Livox ROS2 Driver version %s", LIVOX_ROS_DRIVER2_VERSION_STRING);

  // ── Parameters ────────────────────────────────────────────────────────────
  declare_parameter("xfer_format",      1);          // 1 = CustomMsg (recommended for SLAM)
  declare_parameter("data_src",         0);          // 0 = hardware lidar
  declare_parameter("publish_freq",     10.0);
  declare_parameter("frame_id",         "livox_frame");
  declare_parameter("user_config_path", "");

  int    xfer_format   = get_parameter("xfer_format").as_int();
  int    data_src      = get_parameter("data_src").as_int();
  double publish_freq  = get_parameter("publish_freq").as_double();
  std::string frame_id = get_parameter("frame_id").as_string();
  std::string cfg_path = get_parameter("user_config_path").as_string();

  publish_freq = std::clamp(publish_freq, 0.5, 100.0);

  RCLCPP_INFO(get_logger(), "xfer_format=%d  freq=%.1f Hz  frame=%s",
              xfer_format, publish_freq, frame_id.c_str());
  RCLCPP_INFO(get_logger(), "config: %s", cfg_path.c_str());

  // ── Data distributor ──────────────────────────────────────────────────────
  future_    = exit_signal_.get_future();
  lddc_ptr_  = std::make_unique<Lddc>(xfer_format, data_src, publish_freq, frame_id);
  lddc_ptr_->SetRosNode(this);

  // ── LiDAR init ────────────────────────────────────────────────────────────
  if (data_src == kSourceRawLidar) {
    LdsLidar* read_lidar = LdsLidar::GetInstance(publish_freq);
    lddc_ptr_->RegisterLds(static_cast<Lds*>(read_lidar));
    if (read_lidar->InitLdsLidar(cfg_path)) {
      RCLCPP_INFO(get_logger(), "LiDAR initialized OK");
    } else {
      RCLCPP_ERROR(get_logger(), "LiDAR init FAILED — check config path and network");
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Unsupported data_src=%d", data_src);
  }

  // ── Poll threads (start after 3 s to let the SDK connect) ─────────────────
  pointclouddata_poll_thread_ =
      std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, this);
  imudata_poll_thread_ =
      std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, this);
}

DriverNode::~DriverNode() {
  lddc_ptr_->lds_->RequestExit();
  exit_signal_.set_value();
  if (pointclouddata_poll_thread_->joinable()) pointclouddata_poll_thread_->join();
  if (imudata_poll_thread_->joinable())         imudata_poll_thread_->join();
}

void DriverNode::PointCloudDataPollThread() {
  std::this_thread::sleep_for(std::chrono::seconds(3));
  while (future_.wait_for(std::chrono::microseconds(0)) == std::future_status::timeout) {
    lddc_ptr_->DistributePointCloudData();
  }
}

void DriverNode::ImuDataPollThread() {
  std::this_thread::sleep_for(std::chrono::seconds(3));
  while (future_.wait_for(std::chrono::microseconds(0)) == std::future_status::timeout) {
    lddc_ptr_->DistributeImuData();
  }
}

}  // namespace livox_ros

RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::DriverNode)

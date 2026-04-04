// Copyright (c) 2022 Livox. All rights reserved. MIT License.
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <future>
#include <memory>
#include <thread>

namespace livox_ros {

class Lddc;

class DriverNode final : public rclcpp::Node {
 public:
  explicit DriverNode(const rclcpp::NodeOptions& options);
  DriverNode(const DriverNode&) = delete;
  DriverNode& operator=(const DriverNode&) = delete;
  ~DriverNode();

 private:
  void PointCloudDataPollThread();
  void ImuDataPollThread();

  std::unique_ptr<Lddc>            lddc_ptr_;
  std::shared_ptr<std::thread>     pointclouddata_poll_thread_;
  std::shared_ptr<std::thread>     imudata_poll_thread_;
  std::shared_future<void>         future_;
  std::promise<void>               exit_signal_;
};

}  // namespace livox_ros

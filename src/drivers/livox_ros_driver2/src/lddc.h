// Copyright (c) 2022 Livox. All rights reserved. MIT License.
//
// Trimmed for LingTu / S100P deployment:
//   - ROS2 Humble only  (no ROS1 ifdefs)
//   - Single topic only (multi_topic removed)
//   - CustomMsg output  (PointCloud2 retained, PCL/bag paths removed)

#pragma once

#include "include/livox_ros_driver2.h"
#include "driver_node.h"
#include "lds.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"

namespace livox_ros {

using PublisherPtr  = std::shared_ptr<rclcpp::PublisherBase>;
using PointCloud2   = sensor_msgs::msg::PointCloud2;
using PointField    = sensor_msgs::msg::PointField;
using CustomMsg     = livox_ros_driver2::msg::CustomMsg;
using CustomPoint   = livox_ros_driver2::msg::CustomPoint;
using ImuMsg        = sensor_msgs::msg::Imu;

/** Transfer format (xfer_format parameter) */
enum TransferType : int {
  kPointCloud2Msg = 0,
  kLivoxCustomMsg = 1,
};

/** Output destination */
enum DestinationOfMessageOutput : int {
  kOutputToRos = 0,
};

class DriverNode;

class Lddc final {
 public:
  Lddc(int format, int data_src, double frq, const std::string& frame_id);
  ~Lddc();

  int  RegisterLds(Lds* lds);
  void DistributePointCloudData();
  void DistributeImuData();
  void PrepareExit();

  void SetRosNode(DriverNode* node) { cur_node_ = node; }

 public:
  Lds* lds_ = nullptr;

 private:
  void PollingLidarPointCloudData(uint8_t index, LidarDevice* lidar);
  void PollingLidarImuData(uint8_t index, LidarDevice* lidar);

  /* PointCloud2 path */
  void PublishPointcloud2(LidarDataQueue* queue, uint8_t index);
  void InitPointcloud2MsgHeader(PointCloud2& cloud);
  void InitPointcloud2Msg(const StoragePacket& pkg, PointCloud2& cloud, uint64_t& ts);
  void PublishPointcloud2Data(uint8_t index, uint64_t ts, const PointCloud2& cloud);

  /* CustomMsg path */
  void PublishCustomPointcloud(LidarDataQueue* queue, uint8_t index);
  void InitCustomMsg(CustomMsg& msg, const StoragePacket& pkg, uint8_t index);
  void FillPointsToCustomMsg(CustomMsg& msg, const StoragePacket& pkg);
  void PublishCustomPointData(const CustomMsg& msg, uint8_t index);

  /* IMU */
  void PublishImuData(LidarImuDataQueue& queue, uint8_t index);
  void InitImuMsg(const ImuData& imu_data, ImuMsg& imu_msg, uint64_t& ts);

  PublisherPtr CreatePublisher(uint8_t msg_type, const std::string& topic, uint32_t qsize);
  PublisherPtr GetCurrentPublisher(uint8_t index);
  PublisherPtr GetCurrentImuPublisher(uint8_t index);

 private:
  int         transfer_format_;
  int         data_src_;
  double      publish_frq_;
  std::string frame_id_;

  PublisherPtr lidar_pub_;
  PublisherPtr imu_pub_;

  DriverNode* cur_node_ = nullptr;
};

}  // namespace livox_ros

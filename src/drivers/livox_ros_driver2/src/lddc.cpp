// Copyright (c) 2022 Livox. All rights reserved. MIT License.
//
// Trimmed for LingTu / S100P:
//   - ROS2 only, single topic, CustomMsg + PointCloud2, no PCL, no bag

#include "lddc.h"
#include "comm/ldq.h"
#include "comm/comm.h"
#include "driver_node.h"
#include "lds_lidar.h"

#include <inttypes.h>
#include <memory>

namespace livox_ros {

// ── Constructor / Destructor ──────────────────────────────────────────────────

Lddc::Lddc(int format, int data_src, double frq, const std::string& frame_id)
    : transfer_format_(format),
      data_src_(data_src),
      publish_frq_(frq),
      frame_id_(frame_id) {}

Lddc::~Lddc() {
  PrepareExit();
}

void Lddc::PrepareExit() {
  if (lds_) {
    lds_->PrepareExit();
    lds_ = nullptr;
  }
}

int Lddc::RegisterLds(Lds* lds) {
  if (lds_) return -1;
  lds_ = lds;
  return 0;
}

// ── Distribution threads ──────────────────────────────────────────────────────

void Lddc::DistributePointCloudData() {
  if (!lds_ || lds_->IsRequestExit()) return;
  lds_->pcd_semaphore_.Wait();
  for (uint32_t i = 0; i < lds_->lidar_count_; ++i) {
    LidarDevice* lidar = &lds_->lidars_[i];
    if (kConnectStateSampling != lidar->connect_state) continue;
    PollingLidarPointCloudData(i, lidar);
  }
}

void Lddc::DistributeImuData() {
  if (!lds_ || lds_->IsRequestExit()) return;
  lds_->imu_semaphore_.Wait();
  for (uint32_t i = 0; i < lds_->lidar_count_; ++i) {
    LidarDevice* lidar = &lds_->lidars_[i];
    if (kConnectStateSampling != lidar->connect_state) continue;
    PollingLidarImuData(i, lidar);
  }
}

void Lddc::PollingLidarPointCloudData(uint8_t index, LidarDevice* lidar) {
  LidarDataQueue* q = &lidar->data;
  if (!q || !q->storage_packet) return;
  while (!lds_->IsRequestExit() && !QueueIsEmpty(q)) {
    if (kLivoxCustomMsg == transfer_format_) {
      PublishCustomPointcloud(q, index);
    } else {
      PublishPointcloud2(q, index);
    }
  }
}

void Lddc::PollingLidarImuData(uint8_t index, LidarDevice* lidar) {
  auto& q = lidar->imu_data;
  while (!lds_->IsRequestExit() && !q.Empty()) {
    PublishImuData(q, index);
  }
}

// ── PointCloud2 ───────────────────────────────────────────────────────────────

void Lddc::PublishPointcloud2(LidarDataQueue* queue, uint8_t index) {
  while (!QueueIsEmpty(queue)) {
    StoragePacket pkg;
    QueuePop(queue, &pkg);
    if (pkg.points.empty()) continue;

    PointCloud2 cloud;
    uint64_t ts = 0;
    InitPointcloud2Msg(pkg, cloud, ts);
    PublishPointcloud2Data(index, ts, cloud);
  }
}

void Lddc::InitPointcloud2MsgHeader(PointCloud2& cloud) {
  cloud.header.frame_id = frame_id_;
  cloud.height = 1;
  cloud.width  = 0;
  cloud.fields.resize(7);

  auto set_field = [&](int i, const char* name, uint32_t offset,
                        uint8_t dtype, uint32_t count = 1) {
    cloud.fields[i].name     = name;
    cloud.fields[i].offset   = offset;
    cloud.fields[i].datatype = dtype;
    cloud.fields[i].count    = count;
  };

  set_field(0, "x",         0,  PointField::FLOAT32);
  set_field(1, "y",         4,  PointField::FLOAT32);
  set_field(2, "z",         8,  PointField::FLOAT32);
  set_field(3, "intensity", 12, PointField::FLOAT32);
  set_field(4, "tag",       16, PointField::UINT8);
  set_field(5, "line",      17, PointField::UINT8);
  set_field(6, "timestamp", 18, PointField::FLOAT64);

  cloud.point_step = sizeof(LivoxPointXyzrtlt);
}

void Lddc::InitPointcloud2Msg(const StoragePacket& pkg, PointCloud2& cloud, uint64_t& ts) {
  InitPointcloud2MsgHeader(cloud);
  cloud.width      = pkg.points_num;
  cloud.row_step   = cloud.width * cloud.point_step;
  cloud.is_dense   = true;
  cloud.is_bigendian = false;

  if (!pkg.points.empty()) ts = pkg.base_time;
  cloud.header.stamp = rclcpp::Time(ts);

  cloud.data.resize(pkg.points_num * sizeof(LivoxPointXyzrtlt));
  auto* dst = reinterpret_cast<LivoxPointXyzrtlt*>(cloud.data.data());
  for (size_t i = 0; i < pkg.points_num; ++i) {
    dst[i].x           = pkg.points[i].x;
    dst[i].y           = pkg.points[i].y;
    dst[i].z           = pkg.points[i].z;
    dst[i].reflectivity = static_cast<uint8_t>(pkg.points[i].intensity);
    dst[i].tag         = pkg.points[i].tag;
    dst[i].line        = pkg.points[i].line;
    dst[i].timestamp   = static_cast<double>(pkg.points[i].offset_time);
  }
}

void Lddc::PublishPointcloud2Data(uint8_t index, uint64_t /*ts*/, const PointCloud2& cloud) {
  auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<PointCloud2>>(
      GetCurrentPublisher(index));
  if (pub) pub->publish(cloud);
}

// ── CustomMsg ─────────────────────────────────────────────────────────────────

void Lddc::PublishCustomPointcloud(LidarDataQueue* queue, uint8_t index) {
  while (!QueueIsEmpty(queue)) {
    StoragePacket pkg;
    QueuePop(queue, &pkg);
    if (pkg.points.empty()) continue;

    CustomMsg msg;
    InitCustomMsg(msg, pkg, index);
    FillPointsToCustomMsg(msg, pkg);
    PublishCustomPointData(msg, index);
  }
}

void Lddc::InitCustomMsg(CustomMsg& msg, const StoragePacket& pkg, uint8_t index) {
  msg.header.frame_id = frame_id_;
  uint64_t ts = pkg.points.empty() ? 0 : pkg.base_time;
  msg.timebase  = ts;
  msg.header.stamp = rclcpp::Time(ts);
  msg.point_num = pkg.points_num;
  msg.lidar_id  = (lds_ && lds_->lidars_[index].lidar_type == kLivoxLidarType)
                  ? lds_->lidars_[index].handle : 0;
}

void Lddc::FillPointsToCustomMsg(CustomMsg& msg, const StoragePacket& pkg) {
  msg.points.reserve(pkg.points_num);
  for (uint32_t i = 0; i < pkg.points_num; ++i) {
    CustomPoint p;
    p.x           = pkg.points[i].x;
    p.y           = pkg.points[i].y;
    p.z           = pkg.points[i].z;
    p.reflectivity = static_cast<uint8_t>(pkg.points[i].intensity);
    p.tag         = pkg.points[i].tag;
    p.line        = pkg.points[i].line;
    p.offset_time = static_cast<uint32_t>(pkg.points[i].offset_time - pkg.base_time);
    msg.points.push_back(std::move(p));
  }
}

void Lddc::PublishCustomPointData(const CustomMsg& msg, uint8_t index) {
  auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<CustomMsg>>(
      GetCurrentPublisher(index));
  if (pub) pub->publish(msg);
}

// ── IMU ───────────────────────────────────────────────────────────────────────

void Lddc::PublishImuData(LidarImuDataQueue& queue, uint8_t index) {
  ImuData imu_data;
  if (!queue.Pop(imu_data)) return;

  ImuMsg msg;
  uint64_t ts;
  InitImuMsg(imu_data, msg, ts);

  auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<ImuMsg>>(
      GetCurrentImuPublisher(index));
  if (pub) pub->publish(msg);
}

void Lddc::InitImuMsg(const ImuData& d, ImuMsg& msg, uint64_t& ts) {
  msg.header.frame_id = frame_id_;
  ts = d.time_stamp;
  msg.header.stamp = rclcpp::Time(ts);

  msg.angular_velocity.x    = d.gyro_x;
  msg.angular_velocity.y    = d.gyro_y;
  msg.angular_velocity.z    = d.gyro_z;
  msg.linear_acceleration.x = d.acc_x;
  msg.linear_acceleration.y = d.acc_y;
  msg.linear_acceleration.z = d.acc_z;
}

// ── Publisher management ──────────────────────────────────────────────────────

PublisherPtr Lddc::CreatePublisher(uint8_t msg_type, const std::string& topic, uint32_t qsize) {
  if (kPointCloud2Msg == msg_type) {
    RCLCPP_INFO(cur_node_->get_logger(), "lidar pub → %s [PointCloud2]", topic.c_str());
    return cur_node_->create_publisher<PointCloud2>(topic, qsize);
  } else if (kLivoxCustomMsg == msg_type) {
    RCLCPP_INFO(cur_node_->get_logger(), "lidar pub → %s [CustomMsg]", topic.c_str());
    return cur_node_->create_publisher<CustomMsg>(topic, qsize);
  } else if (kLivoxImuMsg == msg_type) {
    RCLCPP_INFO(cur_node_->get_logger(), "imu pub → %s", topic.c_str());
    return cur_node_->create_publisher<ImuMsg>(topic, qsize);
  }
  return nullptr;
}

PublisherPtr Lddc::GetCurrentPublisher(uint8_t /*index*/) {
  if (!lidar_pub_) {
    lidar_pub_ = CreatePublisher(transfer_format_, "/livox/lidar", 64);
  }
  return lidar_pub_;
}

PublisherPtr Lddc::GetCurrentImuPublisher(uint8_t /*index*/) {
  if (!imu_pub_) {
    imu_pub_ = CreatePublisher(kLivoxImuMsg, "/livox/imu", 64);
  }
  return imu_pub_;
}

}  // namespace livox_ros

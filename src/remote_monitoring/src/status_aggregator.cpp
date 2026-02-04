#include "remote_monitoring/status_aggregator.hpp"

#include <chrono>
#include <cmath>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace remote_monitoring {

void TopicRate::tick() { count_++; }

void TopicRate::reset(double window_sec) {
  if (window_sec > 0.0) {
    rate_hz_ = static_cast<float>(count_ / window_sec);
  } else {
    rate_hz_ = 0.0f;
  }
  count_ = 0;
}

float TopicRate::rate_hz() const { return rate_hz_; }

StatusAggregator::StatusAggregator(rclcpp::Node *node)
  : node_(node),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_) {

  node_->declare_parameter<double>("fast_state_hz", 30.0);
  node_->declare_parameter<double>("slow_state_hz", 1.0);
  node_->declare_parameter<double>("rate_window_sec", 2.0);
  node_->declare_parameter<std::string>("odom_topic", "/Odometry");
  node_->declare_parameter<std::string>("terrain_map_topic", "/terrain_map");
  node_->declare_parameter<std::string>("path_topic", "/path");
  node_->declare_parameter<std::string>("slow_down_topic", "/slow_down");
  node_->declare_parameter<std::string>("tf_map_frame", "map");
  node_->declare_parameter<std::string>("tf_odom_frame", "odom");
  node_->declare_parameter<std::string>("tf_body_frame", "body");

  fast_hz_ = node_->get_parameter("fast_state_hz").as_double();
  slow_hz_ = node_->get_parameter("slow_state_hz").as_double();
  window_sec_ = node_->get_parameter("rate_window_sec").as_double();
  odom_topic_ = node_->get_parameter("odom_topic").as_string();
  terrain_map_topic_ = node_->get_parameter("terrain_map_topic").as_string();
  path_topic_ = node_->get_parameter("path_topic").as_string();
  tf_map_frame_ = node_->get_parameter("tf_map_frame").as_string();
  tf_odom_frame_ = node_->get_parameter("tf_odom_frame").as_string();
  tf_body_frame_ = node_->get_parameter("tf_body_frame").as_string();

  sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&StatusAggregator::OdomCallback, this, std::placeholders::_1));
  sub_terrain_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    terrain_map_topic_, 5, std::bind(&StatusAggregator::TerrainCallback, this, std::placeholders::_1));
  sub_path_ = node_->create_subscription<nav_msgs::msg::Path>(
    path_topic_, 5, std::bind(&StatusAggregator::PathCallback, this, std::placeholders::_1));
  sub_slow_down_ = node_->create_subscription<std_msgs::msg::Int8>(
    node_->get_parameter("slow_down_topic").as_string(), 5,
    std::bind(&StatusAggregator::SlowDownCallback, this, std::placeholders::_1));

  rate_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(window_sec_), [this]() { update_rates(); });
  fast_state_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(fast_hz_, 0.1)), [this]() { update_fast_state(); });
  slow_state_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(slow_hz_, 0.1)), [this]() { update_slow_state(); });
}

void StatusAggregator::OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  odom_rate_.tick();
  latest_odom_ = msg;
}

void StatusAggregator::TerrainCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr) {
  terrain_rate_.tick();
}

void StatusAggregator::PathCallback(const nav_msgs::msg::Path::ConstSharedPtr) {
  path_rate_.tick();
}

void StatusAggregator::SlowDownCallback(const std_msgs::msg::Int8::ConstSharedPtr) {
  // 缓存用于 slow_state
}

void StatusAggregator::update_rates() {
  odom_rate_.reset(window_sec_);
  terrain_rate_.reset(window_sec_);
  path_rate_.reset(window_sec_);
  lidar_rate_.reset(window_sec_);
}

bool StatusAggregator::check_tf(const std::string &target, const std::string &source) {
  try {
    return tf_buffer_.canTransform(target, source, tf2::TimePointZero,
                                   tf2::durationFromSec(0.1));
  } catch (...) {
    return false;
  }
}

robot::v1::FastState StatusAggregator::GetFastState() {
  std::lock_guard<std::mutex> lock(fast_mutex_);
  return latest_fast_state_;
}

robot::v1::SlowState StatusAggregator::GetSlowState() {
  std::lock_guard<std::mutex> lock(slow_mutex_);
  return latest_slow_state_;
}

void StatusAggregator::update_fast_state() {
  robot::v1::FastState state;
  
  // 设置 header
  auto *header = state.mutable_header();
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count();
  header->mutable_timestamp()->set_seconds(now_ns / 1000000000);
  header->mutable_timestamp()->set_nanos(now_ns % 1000000000);
  header->set_frame_id(tf_odom_frame_);
  
  // 从 latest_odom_ 提取位姿和速度
  if (latest_odom_) {
    auto *pose = state.mutable_pose();
    pose->mutable_position()->set_x(latest_odom_->pose.pose.position.x);
    pose->mutable_position()->set_y(latest_odom_->pose.pose.position.y);
    pose->mutable_position()->set_z(latest_odom_->pose.pose.position.z);
    pose->mutable_orientation()->set_x(latest_odom_->pose.pose.orientation.x);
    pose->mutable_orientation()->set_y(latest_odom_->pose.pose.orientation.y);
    pose->mutable_orientation()->set_z(latest_odom_->pose.pose.orientation.z);
    pose->mutable_orientation()->set_w(latest_odom_->pose.pose.orientation.w);
    
    auto *vel = state.mutable_velocity();
    vel->mutable_linear()->set_x(latest_odom_->twist.twist.linear.x);
    vel->mutable_linear()->set_y(latest_odom_->twist.twist.linear.y);
    vel->mutable_linear()->set_z(latest_odom_->twist.twist.linear.z);
    vel->mutable_angular()->set_x(latest_odom_->twist.twist.angular.x);
    vel->mutable_angular()->set_y(latest_odom_->twist.twist.angular.y);
    vel->mutable_angular()->set_z(latest_odom_->twist.twist.angular.z);
    
    // 计算 RPY
    tf2::Quaternion q(latest_odom_->pose.pose.orientation.x,
                      latest_odom_->pose.pose.orientation.y,
                      latest_odom_->pose.pose.orientation.z,
                      latest_odom_->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    state.mutable_rpy_deg()->set_x(roll * 180.0 / M_PI);
    state.mutable_rpy_deg()->set_y(pitch * 180.0 / M_PI);
    state.mutable_rpy_deg()->set_z(yaw * 180.0 / M_PI);
  }
  
  // TF 状态
  const bool tf_ok = check_tf(tf_map_frame_, tf_odom_frame_) &&
                     check_tf(tf_odom_frame_, tf_body_frame_);
  state.set_tf_ok(tf_ok);
  
  std::lock_guard<std::mutex> lock(fast_mutex_);
  latest_fast_state_ = state;
}

void StatusAggregator::update_slow_state() {
  robot::v1::SlowState state;
  
  auto *header = state.mutable_header();
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch()).count();
  header->mutable_timestamp()->set_seconds(now_ns / 1000000000);
  header->mutable_timestamp()->set_nanos(now_ns % 1000000000);
  header->set_frame_id(tf_odom_frame_);
  
  state.set_current_mode("autonomous");  // TODO: 从实际模式读取
  
  // 话题频率
  auto *rates = state.mutable_topic_rates();
  rates->set_odom_hz(odom_rate_.rate_hz());
  rates->set_terrain_map_hz(terrain_rate_.rate_hz());
  rates->set_path_hz(path_rate_.rate_hz());
  rates->set_lidar_hz(lidar_rate_.rate_hz());
  
  std::lock_guard<std::mutex> lock(slow_mutex_);
  latest_slow_state_ = state;
}

}  // namespace remote_monitoring

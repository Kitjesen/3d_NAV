#include "remote_monitoring/core/safety_gate.hpp"

#include <cmath>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace remote_monitoring {
namespace core {

SafetyGate::SafetyGate(rclcpp::Node *node) : node_(node) {
  node_->declare_parameter<double>("deadman_timeout_ms", 300.0);
  node_->declare_parameter<double>("max_speed", 1.0);
  node_->declare_parameter<double>("max_angular", 1.0);
  node_->declare_parameter<double>("tilt_limit_deg", 30.0);
  
  deadman_timeout_ms_ = std::chrono::milliseconds(
    static_cast<int>(node_->get_parameter("deadman_timeout_ms").as_double()));
  max_speed_ = node_->get_parameter("max_speed").as_double();
  max_angular_ = node_->get_parameter("max_angular").as_double();
  tilt_limit_deg_ = node_->get_parameter("tilt_limit_deg").as_double();
  
  sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/Odometry", 10, std::bind(&SafetyGate::OdomCallback, this, std::placeholders::_1));
  
  pub_cmd_vel_safe_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/cmd_vel_safe", 5);
}

robot::v1::Twist SafetyGate::ProcessTeleopCommand(const robot::v1::TeleopCommand &cmd) {
  std::lock_guard<std::mutex> lock(mutex_);
  
  last_teleop_time_ = std::chrono::system_clock::now();
  limit_reasons_.clear();
  
  // 应用限幅
  return ApplyLimits(cmd.target_velocity());
}

robot::v1::SafetyStatus SafetyGate::GetSafetyStatus() {
  std::lock_guard<std::mutex> lock(mutex_);
  
  robot::v1::SafetyStatus status;
  status.set_estop_active(estop_active_);
  
  // 检查 deadman
  const auto now = std::chrono::system_clock::now();
  const bool deadman_active = (now - last_teleop_time_) > deadman_timeout_ms_;
  status.set_deadman_active(deadman_active);
  
  // 检查倾斜
  const bool tilt_limit = std::abs(roll_deg_) > tilt_limit_deg_ ||
                          std::abs(pitch_deg_) > tilt_limit_deg_;
  status.set_tilt_limit_active(tilt_limit);
  
  status.set_speed_limited(!limit_reasons_.empty());
  status.set_max_allowed_speed(max_speed_);
  
  std::string msg;
  for (const auto &reason : limit_reasons_) {
    msg += reason + "; ";
  }
  status.set_safety_message(msg);
  
  return status;
}

std::vector<std::string> SafetyGate::GetLimitReasons() {
  std::lock_guard<std::mutex> lock(mutex_);
  return limit_reasons_;
}

void SafetyGate::SetEmergencyStop(bool active) {
  std::lock_guard<std::mutex> lock(mutex_);
  estop_active_ = active;

  if (active) {
    geometry_msgs::msg::TwistStamped zero;
    zero.header.stamp = node_->now();
    zero.header.frame_id = "body";
    pub_cmd_vel_safe_->publish(zero);
  }
}

void SafetyGate::CheckDeadman() {
  std::lock_guard<std::mutex> lock(mutex_);
  
  const auto now = std::chrono::system_clock::now();
  if ((now - last_teleop_time_) > deadman_timeout_ms_) {
    // 发布零速度
    geometry_msgs::msg::TwistStamped zero;
    zero.header.stamp = node_->now();
    zero.header.frame_id = "body";
    pub_cmd_vel_safe_->publish(zero);
  }
}

void SafetyGate::OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  tf2::Quaternion q(msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  
  std::lock_guard<std::mutex> lock(mutex_);
  roll_deg_ = roll * 180.0 / M_PI;
  pitch_deg_ = pitch * 180.0 / M_PI;
}

robot::v1::Twist SafetyGate::ApplyLimits(const robot::v1::Twist &target) {
  robot::v1::Twist limited = target;
  
  // 速度限幅
  double speed = std::sqrt(target.linear().x() * target.linear().x() +
                           target.linear().y() * target.linear().y());
  if (speed > max_speed_) {
    const double scale = max_speed_ / speed;
    limited.mutable_linear()->set_x(target.linear().x() * scale);
    limited.mutable_linear()->set_y(target.linear().y() * scale);
    limit_reasons_.push_back("max_speed");
  }
  
  // 角速度限幅
  if (std::abs(target.angular().z()) > max_angular_) {
    limited.mutable_angular()->set_z(
      std::copysign(max_angular_, target.angular().z()));
    limit_reasons_.push_back("max_angular");
  }
  
  // 倾斜限制
  if (std::abs(roll_deg_) > tilt_limit_deg_ || std::abs(pitch_deg_) > tilt_limit_deg_) {
    limited.mutable_linear()->set_x(0);
    limited.mutable_linear()->set_y(0);
    limited.mutable_angular()->set_z(0);
    limit_reasons_.push_back("tilt_protection");
  }
  
  // 急停
  if (estop_active_) {
    limited.mutable_linear()->set_x(0);
    limited.mutable_linear()->set_y(0);
    limited.mutable_linear()->set_z(0);
    limited.mutable_angular()->set_x(0);
    limited.mutable_angular()->set_y(0);
    limited.mutable_angular()->set_z(0);
    limit_reasons_.push_back("estop");
  }
  
  // 发布安全速度到 ROS
  geometry_msgs::msg::TwistStamped safe_cmd;
  safe_cmd.header.stamp = node_->now();
  safe_cmd.header.frame_id = "body";
  safe_cmd.twist.linear.x = limited.linear().x();
  safe_cmd.twist.linear.y = limited.linear().y();
  safe_cmd.twist.linear.z = limited.linear().z();
  safe_cmd.twist.angular.x = limited.angular().x();
  safe_cmd.twist.angular.y = limited.angular().y();
  safe_cmd.twist.angular.z = limited.angular().z();
  pub_cmd_vel_safe_->publish(safe_cmd);
  
  return limited;
}

}  // namespace core
}  // namespace remote_monitoring

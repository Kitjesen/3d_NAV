#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "control.pb.h"

namespace remote_monitoring {
namespace core {

class SafetyGate {
public:
  explicit SafetyGate(rclcpp::Node *node);
  
  // 处理遥操作命令（返回限幅后的安全速度）
  robot::v1::Twist ProcessTeleopCommand(const robot::v1::TeleopCommand &cmd);
  
  // 获取当前安全状态
  robot::v1::SafetyStatus GetSafetyStatus();

  // 返回最近一次限幅原因，供 TeleopFeedback 透传。
  std::vector<std::string> GetLimitReasons();

  // 设置急停状态（true: 急停，false: 清除）。
  void SetEmergencyStop(bool active);
  
  // 检查 deadman 超时
  void CheckDeadman();

private:
  void OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  robot::v1::Twist ApplyLimits(const robot::v1::Twist &target);
  
  rclcpp::Node *node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_vel_safe_;
  
  std::mutex mutex_;
  std::chrono::system_clock::time_point last_teleop_time_;
  std::chrono::milliseconds deadman_timeout_ms_{300};
  
  bool estop_active_{false};
  double roll_deg_{0.0};
  double pitch_deg_{0.0};
  double max_speed_{1.0};
  double max_angular_{1.0};
  double tilt_limit_deg_{30.0};
  
  std::vector<std::string> limit_reasons_;
};

}  // namespace core
}  // namespace remote_monitoring

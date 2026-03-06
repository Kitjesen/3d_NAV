/**
 * pct_path_adapter.cpp — ROS2 Thin Shell
 *
 * 核心算法: nav_core::WaypointTracker (pct_adapter_core.hpp)
 * 本文件只负责: ROS2 通信 (pub/sub/TF/参数) + 类型转换
 */
#include <cmath>
#include <chrono>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav_core/pct_adapter_core.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class PCTPathAdapter : public rclcpp::Node
{
public:
  PCTPathAdapter()
  : Node("pct_path_adapter"),
    robot_pos_received_(false)
  {
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/pct_path", 10,
      std::bind(&PCTPathAdapter::path_callback, this, _1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", 10,
      std::bind(&PCTPathAdapter::odom_callback, this, _1));

    waypoint_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/planner_waypoint", 10);

    // 航点跟踪状态 (JSON 格式: {"event":"...","index":N,"total":N})
    // 使用专用话题，避免与全局规划器的 /nav/planner_status (纯字符串) 冲突
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/nav/adapter_status", 10);

    // ── 参数 → WaypointTrackerParams ──
    declare_parameter<double>("waypoint_distance", 0.5);
    declare_parameter<double>("arrival_threshold", 0.5);
    declare_parameter<double>("stuck_timeout_sec", 10.0);
    declare_parameter<int>("max_replan_count", 2);
    declare_parameter<double>("replan_cooldown_sec", 5.0);
    declare_parameter<int>("max_index_jump", 3);
    declare_parameter<double>("max_first_waypoint_dist", 10.0);

    nav_core::WaypointTrackerParams tp;
    tp.waypointDistance   = get_parameter("waypoint_distance").as_double();
    tp.arrivalThreshold   = get_parameter("arrival_threshold").as_double();
    tp.stuckTimeoutSec    = get_parameter("stuck_timeout_sec").as_double();
    tp.maxReplanCount     = get_parameter("max_replan_count").as_int();
    tp.replanCooldownSec  = get_parameter("replan_cooldown_sec").as_double();
    tracker_ = nav_core::WaypointTracker(tp);

    maxIndexJump_ = get_parameter("max_index_jump").as_int();
    maxFirstWaypointDist_ = get_parameter("max_first_waypoint_dist").as_double();

    // 第1级重规划: 重发 goal_pose 触发 PCT 重新规划
    replan_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/nav/goal_pose", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = create_wall_timer(100ms, std::bind(&PCTPathAdapter::control_loop, this));

    RCLCPP_INFO(get_logger(), "PCT Path Adapter initialized (nav_core WaypointTracker)");
  }

private:
  // ── ROS2 类型 → nav_core 类型 ──

  static nav_core::Path rosPathToNavCore(const nav_msgs::msg::Path & msg) {
    nav_core::Path result;
    result.reserve(msg.poses.size());
    for (const auto & ps : msg.poses) {
      nav_core::Pose p;
      p.position.x = ps.pose.position.x;
      p.position.y = ps.pose.position.y;
      p.position.z = ps.pose.position.z;
      result.push_back(p);
    }
    return result;
  }

  // ── 回调 ──

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_pos_ = msg->pose.pose.position;
    odom_frame_ = msg->header.frame_id;
    robot_pos_received_ = true;
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    // R9-2: 零长路径保护 — 发布 FAILED 状态
    if (msg->poses.empty()) {
      RCLCPP_WARN(get_logger(),
        "Received empty path (planning failed), publishing FAILED status.");
      publish_status("failed", 0, 0);
      return;
    }

    // R9-3: 第一个航点距离校验 — 拒绝错误坐标系的路径
    if (robot_pos_received_) {
      double dx = msg->poses[0].pose.position.x - robot_pos_.x;
      double dy = msg->poses[0].pose.position.y - robot_pos_.y;
      double firstDist = std::sqrt(dx * dx + dy * dy);
      if (firstDist > maxFirstWaypointDist_) {
        RCLCPP_WARN(get_logger(),
          "First waypoint is %.1fm from robot (limit=%.1fm), "
          "rejecting path (possible frame mismatch).",
          firstDist, maxFirstWaypointDist_);
        publish_status("failed", 0, 0);
        return;
      }
    }

    // 路径去重: 全局规划器可能定期重发同一条路径 (TRANSIENT_LOCAL republish)
    // 每次 setPath 会重置 tracker 的航点索引 → 机器人回退到先前航点 → 振荡卡死
    // 比较路径长度+终点坐标, 相同则跳过
    if (!tracker_.path().empty() && !tracker_.goalReached() &&
        msg->poses.size() == lastPathSize_) {
      const auto & lastPose = msg->poses.back().pose.position;
      if (std::abs(lastPose.x - lastPathEndX_) < 0.01 &&
          std::abs(lastPose.y - lastPathEndY_) < 0.01) {
        return;  // 同一路径重发, 跳过以保持 tracker 状态
      }
    }

    RCLCPP_INFO(get_logger(),
      "Received new Global Path with %zu points", msg->poses.size());

    auto navPath = rosPathToNavCore(*msg);
    tracker_.setPath(navPath, now().seconds());

    // 记录路径签名用于去重
    lastPathSize_ = msg->poses.size();
    if (!msg->poses.empty()) {
      lastPathEndX_ = msg->poses.back().pose.position.x;
      lastPathEndY_ = msg->poses.back().pose.position.y;
    }

    // R9-4: 路径完整性日志 — 打印总长度和航点数
    double totalLen = 0.0;
    for (size_t i = 1; i < navPath.size(); ++i) {
      double ddx = navPath[i].position.x - navPath[i - 1].position.x;
      double ddy = navPath[i].position.y - navPath[i - 1].position.y;
      totalLen += std::sqrt(ddx * ddx + ddy * ddy);
    }
    RCLCPP_INFO(get_logger(),
      "Path accepted: %zu waypoints, total length %.1fm",
      tracker_.path().size(), totalLen);
    publish_status("path_received", 0, tracker_.path().size());
  }

  // ── TF 变换 ──

  bool transform_robot_to_map(nav_core::Vec3 & out) {
    if (odom_frame_.empty()) return false;

    geometry_msgs::msg::PointStamped input;
    input.header.frame_id = odom_frame_;
    input.header.stamp = rclcpp::Time(0);
    input.point = robot_pos_;

    geometry_msgs::msg::PointStamped output;
    try {
      auto transform = tf_buffer_->lookupTransform(
        "map", odom_frame_, tf2::TimePointZero);
      tf2::doTransform(input, output, transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Could not transform robot to map: %s", ex.what());
      return false;
    }

    out.x = output.point.x;
    out.y = output.point.y;
    out.z = output.point.z;
    return true;
  }

  bool transform_map_to_odom(
    const nav_core::Vec3 & mapPt,
    geometry_msgs::msg::Point & odomPt)
  {
    if (odom_frame_.empty()) return false;

    geometry_msgs::msg::PointStamped input;
    input.header.frame_id = "map";
    input.header.stamp = rclcpp::Time(0);
    input.point.x = mapPt.x;
    input.point.y = mapPt.y;
    input.point.z = mapPt.z;

    geometry_msgs::msg::PointStamped output;
    try {
      auto transform = tf_buffer_->lookupTransform(
        odom_frame_, "map", tf2::TimePointZero);
      tf2::doTransform(input, output, transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Could not transform waypoint: %s", ex.what());
      return false;
    }

    odomPt = output.point;
    return true;
  }

  // ── 控制循环 (Thin Shell — 核心逻辑在 WaypointTracker) ──

  void control_loop()
  {
    if (tracker_.path().empty() || !robot_pos_received_ || odom_frame_.empty()) {
      return;
    }

    // 1. 变换机器人位置到 map 帧 (一次 TF)
    nav_core::Vec3 robotMap;
    if (!transform_robot_to_map(robotMap)) return;

    // 2. 调用核心算法
    size_t prevIdx = tracker_.currentIndex();
    double currentTime = now().seconds();
    auto result = tracker_.update(robotMap, {}, currentTime);

    // R9-1: 航点跳跃保护 — 防止短暂 TF 丢失导致跳过中间航点
    if (result.currentIndex > prevIdx + static_cast<size_t>(maxIndexJump_)) {
      RCLCPP_WARN(get_logger(),
        "Waypoint index jumped %zu→%zu (limit=%d), possible TF glitch",
        prevIdx, result.currentIndex, maxIndexJump_);
    }

    // 3. 处理事件
    switch (result.event) {
      case nav_core::WaypointEvent::kWaypointReached:
        RCLCPP_INFO(get_logger(),
          "Reached Waypoint %zu. Proceeding to next.",
          result.currentIndex);
        publish_status("waypoint_reached",
          static_cast<int>(result.currentIndex),
          static_cast<int>(result.totalWaypoints));
        break;

      case nav_core::WaypointEvent::kGoalReached:
        RCLCPP_INFO(get_logger(), "Goal Reached!");
        publish_status("goal_reached",
          static_cast<int>(result.currentIndex),
          static_cast<int>(result.totalWaypoints));
        break;

      case nav_core::WaypointEvent::kReplanning: {
        auto goal = tracker_.goalPose();
        auto goal_msg = geometry_msgs::msg::PoseStamped();
        goal_msg.header.stamp = now();
        goal_msg.header.frame_id = "map";
        goal_msg.pose.position.x = goal.position.x;
        goal_msg.pose.position.y = goal.position.y;
        goal_msg.pose.position.z = goal.position.z;
        replan_goal_pub_->publish(goal_msg);

        RCLCPP_WARN(get_logger(),
          "[Replan L1] %d replan(s), replanning from current position",
          tracker_.replanCount());
        publish_status("replanning",
          static_cast<int>(result.currentIndex),
          static_cast<int>(result.totalWaypoints));
        break;
      }

      case nav_core::WaypointEvent::kStuckFinal:
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "[Replan] stuck_final after %d replans",
          tracker_.replanCount());
        publish_status("stuck_final",
          static_cast<int>(result.currentIndex),
          static_cast<int>(result.totalWaypoints));
        break;

      default:
        break;
    }

    // 4. Goal 到达后不再发送航点
    if (tracker_.goalReached()) return;

    // 5. 发布当前航点 (map → odom 变换后发布)
    if (result.hasTarget) {
      geometry_msgs::msg::Point targetOdom;
      if (!transform_map_to_odom(result.targetPoint, targetOdom)) return;

      geometry_msgs::msg::PointStamped waypoint_msg;
      waypoint_msg.header.stamp = now();
      waypoint_msg.header.frame_id = odom_frame_;
      waypoint_msg.point = targetOdom;
      waypoint_pub_->publish(waypoint_msg);
    }
  }

  void publish_status(const std::string & event, int index, int total)
  {
    std_msgs::msg::String msg;
    msg.data = "{\"event\":\"" + event + "\","
               "\"index\":" + std::to_string(index) + ","
               "\"total\":" + std::to_string(total) + "}";
    status_pub_->publish(msg);
  }

  // ── ROS2 通信 ──
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr replan_goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── 核心算法 (替代 ~15 个手动成员变量) ──
  nav_core::WaypointTracker tracker_;

  // ── 边界保护参数 ──
  int maxIndexJump_ = 3;
  double maxFirstWaypointDist_ = 10.0;

  // ── 路径去重 ──
  size_t lastPathSize_ = 0;
  double lastPathEndX_ = 0.0;
  double lastPathEndY_ = 0.0;

  // ── ROS2 独有状态 ──
  geometry_msgs::msg::Point robot_pos_;
  bool robot_pos_received_;
  std::string odom_frame_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PCTPathAdapter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

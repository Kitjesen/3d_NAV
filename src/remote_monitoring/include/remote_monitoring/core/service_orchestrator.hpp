#pragma once
/**
 * ServiceOrchestrator — 根据模式切换启停 systemd 服务
 *
 * 模式 → 服务映射:
 *   IDLE:       (保持 lidar+slam 用于监控, 停止 autonomy/planning)
 *   MAPPING:    nav-lidar + nav-slam
 *   AUTONOMOUS: nav-lidar + nav-slam + nav-autonomy + nav-planning
 *   TELEOP:     nav-lidar + nav-slam + nav-autonomy
 *   MANUAL:     nav-lidar + nav-slam
 *   ESTOP:      不动服务, 仅发停车指令
 *
 * 注意: nav-grpc 是自身, 不管理; ota-daemon 独立, 不管理
 */

#include <functional>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "control.pb.h"

namespace remote_monitoring {
namespace core {

class ServiceOrchestrator {
public:
  explicit ServiceOrchestrator(rclcpp::Node *node);

  /// 根据模式转换, 启停对应服务 (异步执行, 不阻塞调用方)
  void OnModeChange(robot::v1::RobotMode old_mode,
                    robot::v1::RobotMode new_mode);

  /// 确保基础服务 (lidar + slam) 正在运行
  void EnsureBaseServices();

  /// 停止所有导航服务
  void StopAllNavServices();

  struct ServiceStatus {
    std::string name;
    std::string state;      // "active", "inactive", "failed"
    std::string sub_state;  // "running", "dead", "failed"
  };

  /// 查询所有受管服务状态
  std::vector<ServiceStatus> GetAllServiceStatuses() const;

private:
  /// 获取指定模式需要的服务集合
  std::set<std::string> GetRequiredServices(robot::v1::RobotMode mode) const;

  /// 同步执行 systemctl 命令, 返回是否成功
  static bool ManageService(const std::string &service,
                            const std::string &action);

  /// 查询单个服务状态
  static std::string QueryServiceField(const std::string &service,
                                       const std::string &field);

  /// 异步编排: 启动需要的, 停止不需要的
  void OrchestrateAsync(std::set<std::string> required);

  rclcpp::Node *node_;
  std::mutex orchestrate_mutex_;

  // 所有可管理的导航服务 (按启动依赖顺序)
  static constexpr const char *kManagedServices[] = {
      "nav-lidar", "nav-slam", "nav-autonomy", "nav-planning"};
  static constexpr size_t kNumManagedServices = 4;
};

}  // namespace core
}  // namespace remote_monitoring

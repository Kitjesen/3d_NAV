#pragma once
/**
 * ServiceOrchestrator — 根据模式切换启停 systemd 服务
 *
 * 模式 → 服务映射:
 *   IDLE:       空集 (进入后 grace period 倒计时, 超时自动停止所有服务)
 *   MAPPING:    nav-lidar + nav-slam
 *   AUTONOMOUS: nav-lidar + nav-slam + nav-autonomy + nav-planning
 *   TELEOP:     nav-lidar + nav-slam + nav-autonomy
 *   MANUAL:     nav-lidar + nav-slam
 *   ESTOP:      不动服务, 仅发停车指令
 *
 * IDLE Grace Period (延迟关停):
 *   切到 IDLE 后启动倒计时 (默认 120 秒):
 *   - 超时前再切到其他模式 → 取消倒计时, 服务仍在运行, 零延迟切换
 *   - 超时后仍在 IDLE   → 自动停止全部服务, 释放 CPU/内存
 *   这避免了频繁切模式时的冷启动开销, 又防止 SLAM 长时间空转吃资源
 *
 * 注意: nav-grpc 是自身, 不管理; ota-daemon 独立, 不管理
 *       SIGSTOP/freeze 对 SLAM 不可用 (IMU 时间跳变导致 EKF 发散)
 */

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "control.pb.h"

namespace remote_monitoring {
namespace core {

class EventBuffer;

/// 服务崩溃回调: (崩溃的服务名, 当前应该跑的模式)
using ServiceCrashCallback =
    std::function<void(const std::string &service_name,
                       robot::v1::RobotMode expected_mode)>;

class ServiceOrchestrator {
public:
  explicit ServiceOrchestrator(rclcpp::Node *node);
  ~ServiceOrchestrator();

  void SetEventBuffer(std::shared_ptr<EventBuffer> eb) {
    event_buffer_ = std::move(eb);
  }

  /// 注入模式查询 (用于定期巡检确定 expected 服务集)
  void SetModeProvider(std::function<robot::v1::RobotMode()> provider) {
    mode_provider_ = std::move(provider);
  }

  /// 注入服务崩溃回调 (通知 ModeManager 降级)
  void SetServiceCrashCallback(ServiceCrashCallback cb) {
    crash_callback_ = std::move(cb);
  }

  /// 设置 IDLE 模式的 grace period (秒). 0 = 立即停止.
  void SetIdleGracePeriod(int seconds) { idle_grace_sec_ = seconds; }

  /// 启动后台巡检线程 (每 interval_sec 秒检查一次)
  void StartPeriodicCheck(int interval_sec = 10);

  /// 停止巡检线程
  void StopPeriodicCheck();

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

  /// 异步编排: 启动需要的, 停止不需要的 (支持取消)
  void OrchestrateAsync(std::set<std::string> required);

  /// 启动单个服务 (含重试, 返回最终是否 active)
  bool StartServiceWithRetry(const std::string &svc, int max_retries = 2);

  /// 后台巡检: 检查 expected 服务是否全部 active
  void PeriodicCheckLoop(int interval_sec);

  // ── IDLE Grace Period 延迟关停 ──
  /// 启动 grace period 倒计时 (到期后停止全部服务)
  void ScheduleIdleShutdown();
  /// 取消尚未到期的 grace period 倒计时
  void CancelIdleShutdown();

  rclcpp::Node *node_;
  std::shared_ptr<EventBuffer> event_buffer_;
  std::mutex orchestrate_mutex_;

  // 编排取消标志: 新模式切换时置 true, OrchestrateAsync 轮询此标志提前退出
  std::atomic<bool> orchestrate_cancel_{false};

  // 巡检线程
  std::thread check_thread_;
  std::atomic<bool> check_stop_{false};

  // IDLE grace period 延迟关停
  int idle_grace_sec_{120};                    // 默认 120 秒
  std::thread idle_shutdown_thread_;
  std::mutex idle_shutdown_mutex_;
  std::condition_variable idle_shutdown_cv_;
  std::atomic<bool> idle_shutdown_cancel_{false};

  // 外部注入
  std::function<robot::v1::RobotMode()> mode_provider_;
  ServiceCrashCallback crash_callback_;

  // 所有可管理的导航服务 (按启动依赖顺序)
  static constexpr const char *kManagedServices[] = {
      "nav-lidar", "nav-slam", "nav-autonomy", "nav-planning"};
  static constexpr size_t kNumManagedServices = 4;
};

}  // namespace core
}  // namespace remote_monitoring

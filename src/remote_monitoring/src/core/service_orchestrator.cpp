#include "remote_monitoring/core/service_orchestrator.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>

namespace remote_monitoring {
namespace core {

// ── 静态常量定义 ──
constexpr const char *ServiceOrchestrator::kManagedServices[];

// ── 辅助: 执行 shell 命令并获取 stdout ──
static std::string ExecCommand(const std::string &cmd) {
  std::array<char, 256> buf;
  std::string result;
  FILE *pipe = popen(cmd.c_str(), "r");
  if (!pipe) return "";
  while (fgets(buf.data(), buf.size(), pipe) != nullptr) {
    result += buf.data();
  }
  pclose(pipe);
  // 去掉末尾换行
  while (!result.empty() &&
         (result.back() == '\n' || result.back() == '\r')) {
    result.pop_back();
  }
  return result;
}

// ================================================================

ServiceOrchestrator::ServiceOrchestrator(rclcpp::Node *node) : node_(node) {
  RCLCPP_INFO(node_->get_logger(),
              "ServiceOrchestrator initialized with %zu managed services",
              kNumManagedServices);
}

std::set<std::string> ServiceOrchestrator::GetRequiredServices(
    robot::v1::RobotMode mode) const {
  switch (mode) {
  case robot::v1::ROBOT_MODE_IDLE:
    // IDLE: 保持 lidar+slam 用于实时监控 (点云/遥测)
    return {"nav-lidar", "nav-slam"};

  case robot::v1::ROBOT_MODE_MAPPING:
    // 建图: 只需 lidar + SLAM
    return {"nav-lidar", "nav-slam"};

  case robot::v1::ROBOT_MODE_MANUAL:
    // 手动: lidar + SLAM (手柄直接控制)
    return {"nav-lidar", "nav-slam"};

  case robot::v1::ROBOT_MODE_TELEOP:
    // 遥操作: 需要地形分析做避障
    return {"nav-lidar", "nav-slam", "nav-autonomy"};

  case robot::v1::ROBOT_MODE_AUTONOMOUS:
    // 全自主: 完整导航栈
    return {"nav-lidar", "nav-slam", "nav-autonomy", "nav-planning"};

  case robot::v1::ROBOT_MODE_ESTOP:
    // 急停: 不改变服务状态, 仅发停车指令
    return {};

  default:
    return {};
  }
}

void ServiceOrchestrator::OnModeChange(robot::v1::RobotMode /*old_mode*/,
                                       robot::v1::RobotMode new_mode) {
  if (new_mode == robot::v1::ROBOT_MODE_ESTOP) {
    // 急停不动服务
    RCLCPP_WARN(node_->get_logger(),
                "ServiceOrchestrator: ESTOP — services untouched");
    return;
  }

  auto required = GetRequiredServices(new_mode);
  if (required.empty() && new_mode != robot::v1::ROBOT_MODE_IDLE) {
    return;
  }

  // 异步执行, 不阻塞 gRPC 线程
  std::thread([this, required = std::move(required)]() {
    OrchestrateAsync(required);
  }).detach();
}

void ServiceOrchestrator::OrchestrateAsync(std::set<std::string> required) {
  std::lock_guard<std::mutex> lock(orchestrate_mutex_);

  // 1. 按依赖顺序启动需要的服务
  for (size_t i = 0; i < kNumManagedServices; ++i) {
    const std::string svc = kManagedServices[i];
    if (required.count(svc)) {
      std::string state = QueryServiceField(svc + ".service", "ActiveState");
      if (state != "active") {
        RCLCPP_INFO(node_->get_logger(),
                    "ServiceOrchestrator: starting %s.service", svc.c_str());
        if (!ManageService(svc + ".service", "start")) {
          RCLCPP_ERROR(node_->get_logger(),
                       "ServiceOrchestrator: FAILED to start %s.service",
                       svc.c_str());
        } else {
          // 等待服务启动 (最多 5 秒)
          for (int retry = 0; retry < 10; ++retry) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            state = QueryServiceField(svc + ".service", "ActiveState");
            if (state == "active") break;
          }
          RCLCPP_INFO(node_->get_logger(),
                      "ServiceOrchestrator: %s.service → %s", svc.c_str(),
                      state.c_str());
        }
      }
    }
  }

  // 2. 按依赖反序停止不需要的服务
  for (int i = static_cast<int>(kNumManagedServices) - 1; i >= 0; --i) {
    const std::string svc = kManagedServices[i];
    if (!required.count(svc)) {
      std::string state = QueryServiceField(svc + ".service", "ActiveState");
      if (state == "active") {
        RCLCPP_INFO(node_->get_logger(),
                    "ServiceOrchestrator: stopping %s.service", svc.c_str());
        ManageService(svc + ".service", "stop");
      }
    }
  }
}

void ServiceOrchestrator::EnsureBaseServices() {
  std::thread([this]() {
    OrchestrateAsync({"nav-lidar", "nav-slam"});
  }).detach();
}

void ServiceOrchestrator::StopAllNavServices() {
  std::thread([this]() {
    OrchestrateAsync({});  // 空集 = 全部停止
  }).detach();
}

bool ServiceOrchestrator::ManageService(const std::string &service,
                                        const std::string &action) {
  // 使用 sudo: sudoers 规则允许 sunrise 用户无密码管理 nav-* 服务
  std::string cmd =
      "sudo /bin/systemctl " + action + " " + service + " 2>&1";
  int ret = std::system(cmd.c_str());
  return ret == 0;
}

std::string ServiceOrchestrator::QueryServiceField(const std::string &service,
                                                   const std::string &field) {
  return ExecCommand("systemctl show -p " + field + " --value " + service +
                     " 2>/dev/null");
}

std::vector<ServiceOrchestrator::ServiceStatus>
ServiceOrchestrator::GetAllServiceStatuses() const {
  std::vector<ServiceStatus> statuses;
  for (size_t i = 0; i < kNumManagedServices; ++i) {
    ServiceStatus s;
    s.name = kManagedServices[i];
    s.state = QueryServiceField(s.name + ".service", "ActiveState");
    s.sub_state = QueryServiceField(s.name + ".service", "SubState");
    statuses.push_back(std::move(s));
  }
  return statuses;
}

}  // namespace core
}  // namespace remote_monitoring

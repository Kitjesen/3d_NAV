#include "remote_monitoring/grpc_gateway.hpp"
#include "remote_monitoring/status_aggregator.hpp"
#include "remote_monitoring/core/lease_manager.hpp"
#include "remote_monitoring/core/event_buffer.hpp"
#include "remote_monitoring/core/safety_gate.hpp"
#include "remote_monitoring/services/system_service.hpp"
#include "remote_monitoring/services/control_service.hpp"
#include "remote_monitoring/services/telemetry_service.hpp"
#include "remote_monitoring/services/data_service.hpp"

namespace remote_monitoring {

GrpcGateway::GrpcGateway(rclcpp::Node *node) : node_(node) {
  node_->declare_parameter<int>("grpc_port", 50051);
  port_ = node_->get_parameter("grpc_port").as_int();
  
  // 创建核心组件
  aggregator_ = std::make_shared<StatusAggregator>(node_);
  lease_mgr_ = std::make_shared<core::LeaseManager>();
  event_buffer_ = std::make_shared<core::EventBuffer>(1000);
  safety_gate_ = std::make_shared<core::SafetyGate>(node_);
  
  // 创建服务实现
  system_service_ = std::make_shared<services::SystemServiceImpl>();
  control_service_ = std::make_shared<services::ControlServiceImpl>(lease_mgr_, safety_gate_);
  telemetry_service_ = std::make_shared<services::TelemetryServiceImpl>(aggregator_, event_buffer_);
  data_service_ = std::make_shared<services::DataServiceImpl>();
}

GrpcGateway::~GrpcGateway() {
  Stop();
}

void GrpcGateway::Start() {
  thread_ = std::thread([this]() { Run(); });
}

void GrpcGateway::Stop() {
  stop_.store(true);
  if (server_) {
    server_->Shutdown();
  }
  if (thread_.joinable()) {
    thread_.join();
  }
}

void GrpcGateway::Run() {
  grpc::ServerBuilder builder;
  builder.AddListeningPort("0.0.0.0:" + std::to_string(port_),
                           grpc::InsecureServerCredentials());
  
  // 注册所有服务
  builder.RegisterService(system_service_.get());
  builder.RegisterService(control_service_.get());
  builder.RegisterService(telemetry_service_.get());
  builder.RegisterService(data_service_.get());
  
  server_ = builder.BuildAndStart();
  RCLCPP_INFO(rclcpp::get_logger("grpc_gateway"), "gRPC Gateway listening on :%d", port_);
  
  // Lease 超时检查
  auto last_check = std::chrono::system_clock::now();
  while (!stop_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    const auto now = std::chrono::system_clock::now();
    if (now - last_check > std::chrono::seconds(1)) {
      lease_mgr_->CheckTimeout();
      safety_gate_->CheckDeadman();
      last_check = now;
    }
  }
}

}  // namespace remote_monitoring

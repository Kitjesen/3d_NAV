#include "remote_monitoring/services/control_service.hpp"
#include "remote_monitoring/core/lease_manager.hpp"
#include "remote_monitoring/core/safety_gate.hpp"

#include <thread>
#include <chrono>

namespace remote_monitoring {
namespace services {

ControlServiceImpl::ControlServiceImpl(
  std::shared_ptr<core::LeaseManager> lease_mgr,
  std::shared_ptr<core::SafetyGate> safety_gate)
  : lease_mgr_(std::move(lease_mgr)),
    safety_gate_(std::move(safety_gate)) {}

grpc::Status ControlServiceImpl::AcquireLease(
  grpc::ServerContext *,
  const robot::v1::AcquireLeaseRequest *request,
  robot::v1::AcquireLeaseResponse *response) {
  
  response->mutable_base()->set_request_id(request->base().request_id());
  
  robot::v1::OperatorLease lease;
  if (lease_mgr_->AcquireLease("client_001", &lease)) {
    *response->mutable_lease() = lease;
    response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  } else {
    response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_LEASE_CONFLICT);
    response->mutable_base()->set_error_message("Lease already held by another client");
  }
  
  return grpc::Status::OK;
}

grpc::Status ControlServiceImpl::RenewLease(
  grpc::ServerContext *,
  const robot::v1::RenewLeaseRequest *request,
  robot::v1::RenewLeaseResponse *response) {
  
  response->mutable_base()->set_request_id(request->base().request_id());
  
  robot::v1::OperatorLease lease;
  if (lease_mgr_->RenewLease(request->lease_token(), &lease)) {
    *response->mutable_lease() = lease;
    response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  } else {
    response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_LEASE_EXPIRED);
    response->mutable_base()->set_error_message("Invalid or expired lease");
  }
  
  return grpc::Status::OK;
}

grpc::Status ControlServiceImpl::ReleaseLease(
  grpc::ServerContext *,
  const robot::v1::ReleaseLeaseRequest *request,
  robot::v1::ReleaseLeaseResponse *response) {
  
  response->mutable_base()->set_request_id(request->base().request_id());
  lease_mgr_->ReleaseLease(request->lease_token());
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  
  return grpc::Status::OK;
}

grpc::Status ControlServiceImpl::SetMode(
  grpc::ServerContext *,
  const robot::v1::SetModeRequest *request,
  robot::v1::SetModeResponse *response) {
  
  response->mutable_base()->set_request_id(request->base().request_id());
  current_mode_.store(request->mode());
  response->set_current_mode(request->mode());
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  
  return grpc::Status::OK;
}

grpc::Status ControlServiceImpl::EmergencyStop(
  grpc::ServerContext *,
  const robot::v1::EmergencyStopRequest *request,
  robot::v1::EmergencyStopResponse *response) {
  
  response->mutable_base()->set_request_id(request->base().request_id());
  // TODO: 触发实际急停
  response->set_stopped(true);
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  
  return grpc::Status::OK;
}

grpc::Status ControlServiceImpl::StreamTeleop(
  grpc::ServerContext *context,
  grpc::ServerReaderWriter<robot::v1::TeleopFeedback, robot::v1::TeleopCommand> *stream) {
  
  robot::v1::TeleopCommand cmd;
  while (stream->Read(&cmd)) {
    // 验证租约
    if (!lease_mgr_->ValidateLease(cmd.lease_token())) {
      robot::v1::TeleopFeedback feedback;
      feedback.mutable_safety_status()->set_safety_message("Invalid lease");
      stream->Write(feedback);
      return grpc::Status(grpc::PERMISSION_DENIED, "Lease required");
    }
    
    // 通过 Safety Gate 处理
    const auto limited_vel = safety_gate_->ProcessTeleopCommand(cmd);
    const auto safety_status = safety_gate_->GetSafetyStatus();
    
    robot::v1::TeleopFeedback feedback;
    feedback.mutable_timestamp()->CopyFrom(cmd.timestamp());
    feedback.set_command_sequence(cmd.sequence());
    *feedback.mutable_actual_velocity() = limited_vel;
    *feedback.mutable_safety_status() = safety_status;
    
    stream->Write(feedback);
  }
  
  return grpc::Status::OK;
}

grpc::Status ControlServiceImpl::StartTask(
  grpc::ServerContext *,
  const robot::v1::StartTaskRequest *request,
  robot::v1::StartTaskResponse *response) {
  
  response->mutable_base()->set_request_id(request->base().request_id());
  response->set_task_id("task_001");  // TODO: 实际任务管理
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  
  return grpc::Status::OK;
}

grpc::Status ControlServiceImpl::CancelTask(
  grpc::ServerContext *,
  const robot::v1::CancelTaskRequest *request,
  robot::v1::CancelTaskResponse *response) {
  
  response->mutable_base()->set_request_id(request->base().request_id());
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  
  return grpc::Status::OK;
}

}  // namespace services
}  // namespace remote_monitoring

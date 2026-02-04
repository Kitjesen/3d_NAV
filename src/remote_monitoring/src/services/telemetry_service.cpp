#include "remote_monitoring/services/telemetry_service.hpp"
#include "remote_monitoring/status_aggregator.hpp"
#include "remote_monitoring/core/event_buffer.hpp"

#include <thread>
#include <chrono>

namespace remote_monitoring {
namespace services {

TelemetryServiceImpl::TelemetryServiceImpl(
  std::shared_ptr<StatusAggregator> aggregator,
  std::shared_ptr<core::EventBuffer> event_buffer)
  : aggregator_(std::move(aggregator)),
    event_buffer_(std::move(event_buffer)) {}

grpc::Status TelemetryServiceImpl::StreamFastState(
  grpc::ServerContext *context,
  const robot::v1::FastStateRequest *request,
  grpc::ServerWriter<robot::v1::FastState> *writer) {
  
  double hz = request->desired_hz() > 0 ? request->desired_hz() : aggregator_->fast_state_hz();
  if (hz > 60.0) hz = 60.0;  // 限制最大频率
  const double period = 1.0 / hz;
  
  while (!context->IsCancelled()) {
    const auto state = aggregator_->GetFastState();
    if (!writer->Write(state)) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(period));
  }
  
  return grpc::Status::OK;
}

grpc::Status TelemetryServiceImpl::StreamSlowState(
  grpc::ServerContext *context,
  const robot::v1::SlowStateRequest *,
  grpc::ServerWriter<robot::v1::SlowState> *writer) {
  
  const double period = 1.0 / aggregator_->slow_state_hz();
  
  while (!context->IsCancelled()) {
    const auto state = aggregator_->GetSlowState();
    if (!writer->Write(state)) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(period));
  }
  
  return grpc::Status::OK;
}

grpc::Status TelemetryServiceImpl::StreamEvents(
  grpc::ServerContext *context,
  const robot::v1::EventStreamRequest *request,
  grpc::ServerWriter<robot::v1::Event> *writer) {
  
  // 先回放历史事件
  const auto history = event_buffer_->GetEventsSince(request->last_event_id());
  for (const auto &event : history) {
    // 过滤
    if (!request->filter_types().empty()) {
      bool match = false;
      for (const auto &type : request->filter_types()) {
        if (event.type() == type) {
          match = true;
          break;
        }
      }
      if (!match) continue;
    }
    
    if (event.severity() < request->min_severity()) {
      continue;
    }
    
    if (!writer->Write(event)) {
      return grpc::Status::OK;
    }
  }
  
  // 然后实时推送新事件（这里简化，实际需要订阅模式）
  while (!context->IsCancelled()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // TODO: 实际应该用回调/队列机制推送新事件
  }
  
  return grpc::Status::OK;
}

grpc::Status TelemetryServiceImpl::AckEvent(
  grpc::ServerContext *,
  const robot::v1::AckEventRequest *request,
  robot::v1::AckEventResponse *response) {
  
  event_buffer_->AckEvent(request->event_id());
  
  response->mutable_base()->set_request_id(request->base().request_id());
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  
  return grpc::Status::OK;
}

}  // namespace services
}  // namespace remote_monitoring

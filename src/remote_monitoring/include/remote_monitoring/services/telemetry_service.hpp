#pragma once

#include <memory>

#include "grpcpp/grpcpp.h"
#include "telemetry.grpc.pb.h"

namespace remote_monitoring {

class StatusAggregator;

namespace core {
class EventBuffer;
}

namespace services {

class TelemetryServiceImpl final : public robot::v1::TelemetryService::Service {
public:
  TelemetryServiceImpl(std::shared_ptr<StatusAggregator> aggregator,
                       std::shared_ptr<core::EventBuffer> event_buffer);
  
  grpc::Status StreamFastState(grpc::ServerContext *context,
                                const robot::v1::FastStateRequest *request,
                                grpc::ServerWriter<robot::v1::FastState> *writer) override;
  
  grpc::Status StreamSlowState(grpc::ServerContext *context,
                                const robot::v1::SlowStateRequest *request,
                                grpc::ServerWriter<robot::v1::SlowState> *writer) override;
  
  grpc::Status StreamEvents(grpc::ServerContext *context,
                            const robot::v1::EventStreamRequest *request,
                            grpc::ServerWriter<robot::v1::Event> *writer) override;
  
  grpc::Status AckEvent(grpc::ServerContext *context,
                        const robot::v1::AckEventRequest *request,
                        robot::v1::AckEventResponse *response) override;

private:
  std::shared_ptr<StatusAggregator> aggregator_;
  std::shared_ptr<core::EventBuffer> event_buffer_;
};

}  // namespace services
}  // namespace remote_monitoring

#pragma once

#include "grpcpp/grpcpp.h"
#include "system.grpc.pb.h"

namespace remote_monitoring {
namespace services {

class SystemServiceImpl final : public robot::v1::SystemService::Service {
public:
  SystemServiceImpl();
  
  grpc::Status Login(grpc::ServerContext *context,
                     const robot::v1::LoginRequest *request,
                     robot::v1::LoginResponse *response) override;
  
  grpc::Status Logout(grpc::ServerContext *context,
                      const robot::v1::LogoutRequest *request,
                      robot::v1::LogoutResponse *response) override;
  
  grpc::Status Heartbeat(grpc::ServerContext *context,
                         const robot::v1::HeartbeatRequest *request,
                         robot::v1::HeartbeatResponse *response) override;
  
  grpc::Status GetRobotInfo(grpc::ServerContext *context,
                            const google::protobuf::Empty *request,
                            robot::v1::RobotInfoResponse *response) override;
  
  grpc::Status GetCapabilities(grpc::ServerContext *context,
                               const google::protobuf::Empty *request,
                               robot::v1::CapabilitiesResponse *response) override;

private:
  std::string robot_id_{"robot_001"};
  std::string firmware_version_{"1.0.0"};
};

}  // namespace services
}  // namespace remote_monitoring

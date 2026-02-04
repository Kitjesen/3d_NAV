#pragma once

#include "grpcpp/grpcpp.h"
#include "data.grpc.pb.h"

namespace remote_monitoring {
namespace services {

class DataServiceImpl final : public robot::v1::DataService::Service {
public:
  DataServiceImpl();
  
  grpc::Status ListResources(grpc::ServerContext *context,
                             const google::protobuf::Empty *request,
                             robot::v1::ListResourcesResponse *response) override;
  
  grpc::Status Subscribe(grpc::ServerContext *context,
                         const robot::v1::SubscribeRequest *request,
                         grpc::ServerWriter<robot::v1::DataChunk> *writer) override;
  
  grpc::Status Unsubscribe(grpc::ServerContext *context,
                           const robot::v1::UnsubscribeRequest *request,
                           robot::v1::UnsubscribeResponse *response) override;
  
  grpc::Status DownloadFile(grpc::ServerContext *context,
                            const robot::v1::DownloadFileRequest *request,
                            grpc::ServerWriter<robot::v1::FileChunk> *writer) override;
  
  grpc::Status StartCamera(grpc::ServerContext *context,
                           const robot::v1::StartCameraRequest *request,
                           robot::v1::StartCameraResponse *response) override;
  
  grpc::Status StopCamera(grpc::ServerContext *context,
                          const robot::v1::StopCameraRequest *request,
                          robot::v1::StopCameraResponse *response) override;
};

}  // namespace services
}  // namespace remote_monitoring

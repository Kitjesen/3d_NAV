#include "remote_monitoring/services/data_service.hpp"

namespace remote_monitoring {
namespace services {

DataServiceImpl::DataServiceImpl() {}

grpc::Status DataServiceImpl::ListResources(
  grpc::ServerContext *,
  const google::protobuf::Empty *,
  robot::v1::ListResourcesResponse *response) {
  
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  
  // 添加示例资源
  auto *res1 = response->add_resources();
  res1->mutable_id()->set_type(robot::v1::RESOURCE_TYPE_CAMERA);
  res1->mutable_id()->set_name("front");
  res1->set_description("Front camera");
  res1->set_available(true);
  
  return grpc::Status::OK;
}

grpc::Status DataServiceImpl::Subscribe(
  grpc::ServerContext *context,
  const robot::v1::SubscribeRequest *,
  grpc::ServerWriter<robot::v1::DataChunk> *) {
  
  // TODO: 实现点云/地图订阅（Phase 4）
  return grpc::Status(grpc::UNIMPLEMENTED, "Not implemented yet");
}

grpc::Status DataServiceImpl::Unsubscribe(
  grpc::ServerContext *,
  const robot::v1::UnsubscribeRequest *request,
  robot::v1::UnsubscribeResponse *response) {
  
  response->mutable_base()->set_request_id(request->base().request_id());
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  
  return grpc::Status::OK;
}

grpc::Status DataServiceImpl::DownloadFile(
  grpc::ServerContext *,
  const robot::v1::DownloadFileRequest *,
  grpc::ServerWriter<robot::v1::FileChunk> *) {
  
  // TODO: 实现文件下载（Phase 4）
  return grpc::Status(grpc::UNIMPLEMENTED, "Not implemented yet");
}

grpc::Status DataServiceImpl::StartCamera(
  grpc::ServerContext *,
  const robot::v1::StartCameraRequest *request,
  robot::v1::StartCameraResponse *response) {
  
  response->mutable_base()->set_request_id(request->base().request_id());
  // TODO: WebRTC 信令（Phase 3）
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  response->set_session_id("camera_session_001");
  
  return grpc::Status::OK;
}

grpc::Status DataServiceImpl::StopCamera(
  grpc::ServerContext *,
  const robot::v1::StopCameraRequest *request,
  robot::v1::StopCameraResponse *response) {
  
  response->mutable_base()->set_request_id(request->base().request_id());
  response->mutable_base()->set_error_code(robot::v1::ERROR_CODE_OK);
  
  return grpc::Status::OK;
}

}  // namespace services
}  // namespace remote_monitoring

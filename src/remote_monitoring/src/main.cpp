#include "rclcpp/rclcpp.hpp"
#include "remote_monitoring/grpc_gateway.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("grpc_gateway");
  
  auto gateway = std::make_shared<remote_monitoring::GrpcGateway>(node.get());
  gateway->Start();
  
  rclcpp::spin(node);
  
  gateway->Stop();
  rclcpp::shutdown();
  return 0;
}

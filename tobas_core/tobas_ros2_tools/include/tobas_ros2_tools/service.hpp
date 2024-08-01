#pragma once

#include <rclcpp/rclcpp.hpp>

namespace ros2
{
template <typename SrvType>
class ServiceClientWrapper
{
public:
  SrvType::Request req;
  SrvType::Response res;

  explicit ServiceClientWrapper(rclcpp::Node::SharedPtr node, const std::string& service_name) : node_(node)
  {
    client_ = node->create_client<SrvType>(service_name);
  }

  bool wait_for_service(double timeout = -1)
  {
    return client_.wait_for_service(timeout);
  }

  bool call()
  {
    res = client_.anync_send_request(req);
    return rclcpp::spin_until_future_complete(node, res) == rclcpp::FutureReturnCode::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<ServiceT>::SharedPtr client_;
};
}  // namespace ros2

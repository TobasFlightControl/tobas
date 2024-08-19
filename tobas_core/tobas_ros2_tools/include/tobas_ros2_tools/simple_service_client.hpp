#pragma once

#include <rclcpp/rclcpp.hpp>

namespace ros2
{
template <typename SrvType>
class SimpleServiceClient
{
public:
  using SharedPtr = std::shared_ptr<SimpleServiceClient>;

  inline explicit SimpleServiceClient(rclcpp::Node::SharedPtr node, const std::string& srv_name) : node_(node)
  {
    client_ = node_->create_client<SrvType>(srv_name);
  }

  bool call(const SrvType::Request::SharedPtr& req)
  {
    if (!client_->service_is_ready())
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "\"" << client_->get_service_name() << "\" service is not ready.");
      return false;
    }

    auto id = client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node_, id) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to call \"" << client_->get_service_name() << "\" service.");
      return false;
    }
    res_ = id.get();

    return true;
  }

  inline const SrvType::Response::SharedPtr& getResponse() const
  {
    return res_;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<SrvType>::SharedPtr client_;
  SrvType::Response::SharedPtr res_;
};
}  // namespace ros2

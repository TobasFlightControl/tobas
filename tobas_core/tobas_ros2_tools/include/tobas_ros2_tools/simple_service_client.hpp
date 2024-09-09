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
    client_ = node->create_client<SrvType>(srv_name);
  }

  template <typename RepType = int64_t, typename DurType = std::ratio<1L>>
  bool call(
    const SrvType::Request::SharedPtr& req,
    std::chrono::duration<RepType, DurType> timeout = std::chrono::seconds(0))
  {
    if (!client_->service_is_ready())
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "\"" << client_->get_service_name() << "\" service is not ready.");
      return false;
    }

    auto future = client_->async_send_request(req);

    if (timeout.count() > 0)
    {
      if (future.wait_for(timeout) == std::future_status::timeout)
      {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Timeout before \"" << client_->get_service_name() << "\" response.");
        return false;
      }
    }
    else
    {
      future.wait();
    }

    res_ = future.get();

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

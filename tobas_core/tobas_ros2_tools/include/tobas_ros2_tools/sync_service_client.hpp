#pragma once

#include <rclcpp/rclcpp.hpp>

// #include <std_srvs/srv/empty.hpp>

#include "./future.hpp"

namespace ros2
{
// using SrvType = std_srvs::srv::Empty;
template <typename SrvType>
class SyncServiceClient
{
public:
  using SharedPtr = std::shared_ptr<SyncServiceClient>;

  inline explicit SyncServiceClient(
    rclcpp::Node::SharedPtr node,
    const std::string& name,
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
    : node_(node)
  {
    client_ = node->create_client<SrvType>(name, rclcpp::ServicesQoS(), group);
  }

  /**
   * @brief サービスを呼び，結果が得られるまで待機する．
   *
   * @param req サービスリクエスト．
   * @param timeout レスポンスが得られるまでのタイムアウト．非正ならば無限待機．
   *
   * @note ROSノードと同じスレッドで動作するコールバックの中で呼ぶとデッドロックする．
   */
  bool call(const SrvType::Request::SharedPtr& req, std::chrono::milliseconds timeout = std::chrono::milliseconds(-1))
  {
    if (!client_->service_is_ready())
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "\"" << client_->get_service_name() << "\" service is not ready.");
      return false;
    }

    auto future = client_->async_send_request(req);
    if (waitForFuture(future, timeout) != std::future_status::ready)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Timeout before \"" << client_->get_service_name() << "\" response.");
      return false;
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

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/rclcpp.hpp>

/* For development. */
// #include <std_srvs/srv/empty.hpp>
// using SrvType = std_srvs::srv::Empty;

#include "./future.hpp"

namespace tobas
{
namespace ros2
{
/**
 * @brief Synchronous service client.
 * @note This blocks, so do not use it in nodes where real-time behavior is important.
 */
template <typename SrvType>
class SyncServiceClient
{
  static constexpr auto kWaitForServer = std::chrono::seconds(1);

public:
  using SharedPtr = std::shared_ptr<SyncServiceClient>;

  explicit SyncServiceClient(
    rclcpp::Node::SharedPtr node,
    const std::string& name,
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
    : node_(node)
  {
    client_ = node->create_client<SrvType>(name, rclcpp::ServicesQoS(), group);
  }

  /**
   * @brief Call the service and wait until a result is obtained.
   *
   * @param req Service request.
   * @param timeout Timeout until a response is obtained. Waits indefinitely when non-positive.
   *
   * @note Calling this from a callback that runs on the same thread as the ROS node causes a deadlock.
   */
  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool call(
    const typename SrvType::Request::SharedPtr& req,
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1))
  {
    if (!client_->wait_for_service(kWaitForServer))  // `service_is_ready` often returns false on the first call.
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "\"" << client_->get_service_name() << "\" service is not ready.");
      return false;
    }

    auto future = client_->async_send_request(req);
    if (waitForFuture(future, timeout) != std::future_status::ready) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Timeout before \"" << client_->get_service_name() << "\" response.");
      return false;
    }

    res_ = future.get();

    return true;
  }

  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool waitForService(std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1))
  {
    return client_->wait_for_service(timeout);
  }

  inline typename SrvType::Response::SharedPtr getResponse() const
  {
    return res_;
  }

private:
  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Client<SrvType>::SharedPtr client_;
  typename SrvType::Response::SharedPtr res_;
};
}  // namespace ros2
}  // namespace tobas

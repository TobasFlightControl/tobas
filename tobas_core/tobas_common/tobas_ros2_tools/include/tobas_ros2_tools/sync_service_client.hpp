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
 * @note Service calls block the calling thread. Do not use this client where real-time behavior is required.
 */
template <typename SrvType>
class SyncServiceClient
{
public:
  using SharedPtr = std::shared_ptr<SyncServiceClient>;

  explicit SyncServiceClient(
    rclcpp::Node::SharedPtr node,
    const std::string& name,
    rclcpp::CallbackGroup::SharedPtr group = nullptr);

  /**
   * @brief Call the service and wait for a response.
   *
   * @param req Service request.
   * @param timeout Maximum time to wait for a response. Waits indefinitely when non-positive.
   *
   * @note Calling this from a callback that runs on the same thread as the ROS node causes a deadlock.
   */
  template <typename RepT = int64_t, typename RatioT = std::milli>
  typename SrvType::Response::ConstSharedPtr sendRequestAndWait(
    const typename SrvType::Request::SharedPtr& req,
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1));

  bool serviceIsReady() const;

  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool waitForService(std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1));

private:
  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Client<SrvType>::SharedPtr client_;
};

template <typename SrvType>
SyncServiceClient<SrvType>::SyncServiceClient(
  rclcpp::Node::SharedPtr node,
  const std::string& name,
  rclcpp::CallbackGroup::SharedPtr group)
  : node_(node)
{
  client_ = node->create_client<SrvType>(name, rclcpp::ServicesQoS(), group);
}

template <typename SrvType>
template <typename RepT, typename RatioT>
typename SrvType::Response::ConstSharedPtr SyncServiceClient<SrvType>::sendRequestAndWait(
  const typename SrvType::Request::SharedPtr& req,
  std::chrono::duration<RepT, RatioT> timeout)
{
  // The service may not be ready immediately after client creation because DDS discovery may still be in progress.
  if (!client_->service_is_ready()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "\"" << client_->get_service_name() << "\" service is not ready.");
    return nullptr;
  }

  // Send the request and wait for the response.
  auto future = client_->async_send_request(req);
  if (waitForFuture(future, timeout) != std::future_status::ready) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Timeout before \"" << client_->get_service_name() << "\" response.");
    return nullptr;
  }

  return future.get();
}

template <typename SrvType>
bool SyncServiceClient<SrvType>::serviceIsReady() const
{
  return client_->service_is_ready();
}

template <typename SrvType>
template <typename RepT, typename RatioT>
bool SyncServiceClient<SrvType>::waitForService(std::chrono::duration<RepT, RatioT> timeout)
{
  return client_->wait_for_service(timeout);
}
}  // namespace ros2
}  // namespace tobas

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>

/* For development */
// #include <std_srvs/srv/empty.hpp>
// using SrvType = std_srvs::srv::Empty;

#include "./future.hpp"

namespace tobas::ros2
{
/**
 * Synchronous service client.
 *
 * @note Service calls block the calling thread. Do not use this client where real-time behavior is required.
 */
template <typename SrvType>
class SyncServiceClient
{
public:
  explicit SyncServiceClient(
    rclcpp::Node::SharedPtr _node,
    const std::string& _name,
    rclcpp::CallbackGroup::SharedPtr _group = nullptr);

  /**
   * Call the service and wait for a response.
   *
   * @param _req Service request.
   * @param _timeout Maximum time to wait for a response. Waits indefinitely when non-positive.
   *
   * @note Calling this from a callback that runs on the same thread as the ROS node causes a deadlock.
   */
  template <typename RepT = int64_t, typename RatioT = std::milli>
  typename SrvType::Response::ConstSharedPtr sendRequestAndWait(
    const typename SrvType::Request::SharedPtr& _req,
    std::chrono::duration<RepT, RatioT> _timeout = std::chrono::duration<RepT, RatioT>(-1));

  inline bool serviceIsReady() const;

  template <typename RepT = int64_t, typename RatioT = std::milli>
  inline bool waitForService(std::chrono::duration<RepT, RatioT> _timeout = std::chrono::duration<RepT, RatioT>(-1));

private:
  const rclcpp::Node::SharedPtr node_;
  typename rclcpp::Client<SrvType>::SharedPtr client_;
};

template <typename SrvType>
SyncServiceClient<SrvType>::SyncServiceClient(
  rclcpp::Node::SharedPtr _node,
  const std::string& _name,
  rclcpp::CallbackGroup::SharedPtr _group)
  : node_(_node)
{
  client_ = _node->create_client<SrvType>(_name, rclcpp::ServicesQoS(), _group);
}

template <typename SrvType>
template <typename RepT, typename RatioT>
typename SrvType::Response::ConstSharedPtr SyncServiceClient<SrvType>::sendRequestAndWait(
  const typename SrvType::Request::SharedPtr& _req,
  std::chrono::duration<RepT, RatioT> _timeout)
{
  // The service may not be ready immediately after client creation because DDS discovery may still be in progress.
  if (!client_->service_is_ready()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "'" << client_->get_service_name() << "' service is not ready.");
    return nullptr;
  }

  // Send the request and wait for the response.
  auto future = client_->async_send_request(_req);
  if (waitForFuture(future, _timeout) != std::future_status::ready) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Timeout before '" << client_->get_service_name() << "' response.");
    return nullptr;
  }

  return future.get();
}

template <typename SrvType>
inline bool SyncServiceClient<SrvType>::serviceIsReady() const
{
  return client_->service_is_ready();
}

template <typename SrvType>
template <typename RepT, typename RatioT>
inline bool SyncServiceClient<SrvType>::waitForService(std::chrono::duration<RepT, RatioT> _timeout)
{
  return client_->wait_for_service(_timeout);
}
}  // namespace tobas::ros2

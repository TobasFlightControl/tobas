// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "./future.hpp"

namespace tobas
{
namespace ros2
{
/**
 * @brief Synchronous parameter client.
 * @note Service calls block the calling thread. Do not use this client where real-time behavior is required.
 */
class SyncParamClient
{
public:
  using SharedPtr = std::shared_ptr<SyncParamClient>;

  enum ErrorCode
  {
    kNoError,
    kServerNotReady,
    kTimeout,
    kResultSizeMismatch,
    kServerError,
  };

  inline explicit SyncParamClient(rclcpp::Node::SharedPtr node, const std::string& remote_node_name);

  template <typename ValueT, typename RepT = int64_t, typename RatioT = std::milli>
  ErrorCode setParam(
    const std::string& param_name,
    const ValueT& value,
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1));

  template <typename RepT = int64_t, typename RatioT = std::milli>
  bool waitForService(std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1));

  inline ErrorCode errorCode() const;
  inline const char* errorMessage() const;

private:
  const rclcpp::Node::SharedPtr node_;
  const std::string remote_node_name_;

  // Asynchronous parameter client.
  // The synchronous version spins, so it cannot be used on an executor.
  rclcpp::AsyncParametersClient client_;

  ErrorCode error_code_ = kNoError;
  std::string server_error_msg_;
};

inline SyncParamClient::SyncParamClient(rclcpp::Node::SharedPtr node, const std::string& remote_node_name)
  : node_(node), remote_node_name_(remote_node_name), client_(node, remote_node_name)
{
}

template <typename ValueT, typename RepT, typename RatioT>
SyncParamClient::ErrorCode
SyncParamClient::setParam(const std::string& param_name, const ValueT& value, std::chrono::duration<RepT, RatioT> timeout)
{
  if (!client_.service_is_ready()) {
    return error_code_ = kServerNotReady;
  }

  auto future = client_.set_parameters({ rclcpp::Parameter(param_name, value) });
  if (waitForFuture(future, timeout) != std::future_status::ready) {
    return error_code_ = kTimeout;
  }

  const auto results = future.get();
  if (results.size() != 1) {
    return error_code_ = kResultSizeMismatch;
  }

  const auto res = results.front();
  if (!res.successful) {
    server_error_msg_ = res.reason;
    return error_code_ = kServerError;
  }

  return error_code_ = kNoError;
}

template <typename RepT, typename RatioT>
bool SyncParamClient::waitForService(std::chrono::duration<RepT, RatioT> timeout)
{
  return client_.wait_for_service(timeout);
}

inline SyncParamClient::ErrorCode SyncParamClient::errorCode() const
{
  return error_code_;
}

inline const char* SyncParamClient::errorMessage() const
{
  switch (error_code_) {
    case kNoError:
      return "No error.";
    case kServerNotReady:
      return "The parameter server is not ready";
    case kTimeout:
      return "Timeout before setting the parameter.";
    case kResultSizeMismatch:
      return "Result size mismatch.";
    case kServerError:
      return server_error_msg_.c_str();
    default:
      return "Unexpected error.";
  }
}
}  // namespace ros2
}  // namespace tobas

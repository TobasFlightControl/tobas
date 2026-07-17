// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>

#include <tobas_dparam_msgs/srv/set_bool.hpp>
#include <tobas_dparam_msgs/srv/set_double.hpp>
#include <tobas_dparam_msgs/srv/set_int.hpp>
#include <tobas_dparam_msgs/srv/set_string.hpp>

namespace tobas
{
namespace dparam
{
class DynamicParamClient
{
public:
  using SharedPtr = std::shared_ptr<DynamicParamClient>;

  enum Error
  {
    kNoError = 0,
    kServiceNotReady = -1,
    kServerError = -2,
  };

  explicit DynamicParamClient(rclcpp::Node::SharedPtr node, const std::string& node_name, const std::string& ns = "");

  Error setBool(const std::string& param_name, const bool& value);
  Error setInt(const std::string& param_name, const long& value);
  Error setDouble(const std::string& param_name, const long& value);
  Error setString(const std::string& param_name, const std::string& value);

  Error errorCode() const;
  const char* errorMessage() const;

private:
  const rclcpp::Node::SharedPtr node_;
  const std::string node_name_;

  ros2::SyncServiceClient<tobas_dparam_msgs::srv::SetBool> bool_sc_;
  ros2::SyncServiceClient<tobas_dparam_msgs::srv::SetInt> int_sc_;
  ros2::SyncServiceClient<tobas_dparam_msgs::srv::SetDouble> double_sc_;
  ros2::SyncServiceClient<tobas_dparam_msgs::srv::SetString> string_sc_;

  Error error_code_ = kNoError;
  std::string server_error_msg_;

  template <typename SrvType, typename T>
  Error setParam(ros2::SyncServiceClient<SrvType>& sc, const std::string& param_name, T& value);
};

template <typename SrvType, typename T>
DynamicParamClient::Error
DynamicParamClient::setParam(ros2::SyncServiceClient<SrvType>& sc, const std::string& param_name, T& value)
{
  const auto req = std::make_shared<typename SrvType::Request>();
  req->node_name = node_name_;
  req->param_name = param_name;
  req->value = value;

  const auto res = sc.sendRequestAndWait(req);
  if (!res) {
    return error_code_ = kServiceNotReady;
  }

  if (!res->success) {
    server_error_msg_ = res->message;
    return error_code_ = kServerError;
  }

  return error_code_ = kNoError;
}
}  // namespace dparam
}  // namespace tobas

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <tobas_property_common/constants.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <tobas_property_msgs/srv/get_bool.hpp>
#include <tobas_property_msgs/srv/get_double.hpp>
#include <tobas_property_msgs/srv/get_int.hpp>
#include <tobas_property_msgs/srv/get_string.hpp>
#include <tobas_property_msgs/srv/set_bool.hpp>
#include <tobas_property_msgs/srv/set_double.hpp>
#include <tobas_property_msgs/srv/set_int.hpp>
#include <tobas_property_msgs/srv/set_string.hpp>

namespace tobas
{
namespace ptree
{
/**
 * @brief Client for the property server.
 * @note Calling this from a callback that runs on the same thread as the ROS node causes a deadlock.
 */
class PropertyClient
{
public:
  using SharedPtr = std::shared_ptr<PropertyClient>;

  enum Error
  {
    kNoError = 0,
    kServiceNotReady = -1,
    kOutputRange = -2,
    kServerError = -3,
  };

  explicit PropertyClient(rclcpp::Node::SharedPtr node, const std::string& section = "DEFAULT");

  Error get(const std::string& key, bool& value);
  Error get(const std::string& key, int& value);
  Error get(const std::string& key, double& value);
  Error get(const std::string& key, std::string& value);

  Error get(const std::string& key, uint8_t& value);
  Error get(const std::string& key, uint16_t& value);
  Error get(const std::string& key, float& value);

  Error set(const std::string& key, const bool& value);
  Error set(const std::string& key, const int& value);
  Error set(const std::string& key, const double& value);
  Error set(const std::string& key, const std::string& value);

  Error set(const std::string& key, const uint8_t& value);
  Error set(const std::string& key, const uint16_t& value);
  Error set(const std::string& key, const float& value);

  Error save();

  Error errorCode() const;
  const char* errorMessage() const;

private:
  const rclcpp::Node::SharedPtr node_;
  const std::string section_;

  ros2::SyncServiceClient<tobas_property_msgs::srv::GetBool> get_bool_sc_;
  ros2::SyncServiceClient<tobas_property_msgs::srv::GetInt> get_int_sc_;
  ros2::SyncServiceClient<tobas_property_msgs::srv::GetDouble> get_double_sc_;
  ros2::SyncServiceClient<tobas_property_msgs::srv::GetString> get_string_sc_;
  ros2::SyncServiceClient<tobas_property_msgs::srv::SetBool> set_bool_sc_;
  ros2::SyncServiceClient<tobas_property_msgs::srv::SetInt> set_int_sc_;
  ros2::SyncServiceClient<tobas_property_msgs::srv::SetDouble> set_double_sc_;
  ros2::SyncServiceClient<tobas_property_msgs::srv::SetString> set_string_sc_;
  ros2::SyncServiceClient<std_srvs::srv::Trigger> save_sc_;

  Error error_code_ = kNoError;
  std::string server_error_msg_;

  template <typename SrvType, typename T>
  Error getProperty(ros2::SyncServiceClient<SrvType>& sc, const std::string& key, T& value);

  template <typename SrvType, typename T>
  Error setProperty(ros2::SyncServiceClient<SrvType>& sc, const std::string& key, T& value);
};

template <typename SrvType, typename T>
PropertyClient::Error PropertyClient::getProperty(ros2::SyncServiceClient<SrvType>& sc, const std::string& key, T& value)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Get property requested: " << key);

  const auto req = std::make_shared<typename SrvType::Request>();
  req->section = section_;
  req->key = key;

  const auto res = sc.sendRequestAndWait(req);
  if (!res) {
    return error_code_ = kServiceNotReady;
  }

  if (!res->success) {
    server_error_msg_ = res->message;
    return error_code_ = kServerError;
  }

  value = res->value;

  return error_code_ = kNoError;
}

template <typename SrvType, typename T>
PropertyClient::Error PropertyClient::setProperty(ros2::SyncServiceClient<SrvType>& sc, const std::string& key, T& value)
{
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Set property requested: " << key << ", " << value);

  const auto req = std::make_shared<typename SrvType::Request>();
  req->section = section_;
  req->key = key;
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
}  // namespace ptree
}  // namespace tobas

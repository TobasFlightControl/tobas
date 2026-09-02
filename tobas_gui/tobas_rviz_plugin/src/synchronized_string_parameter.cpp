// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <rclcpp/wait_set.hpp>

#include <QDebug>

#include "tobas_rviz_plugin/synchronized_string_parameter.hpp"

namespace ch = std::chrono;

namespace tobas
{
namespace rviz
{
std::string SynchronizedStringParameter::loadInitialValue(
  const rclcpp::Node::SharedPtr& node,
  const std::string& name,
  const StringCallback& parent_callback,
  bool default_continuous_value,
  double default_timeout)
{
  node_ = node;
  name_ = name;
  parent_callback_ = parent_callback;

  if (getMainParameter()) {
    if (shouldPublish()) {
      // Transient local is similar to latching in ROS 1.
      string_publisher_ = node_->create_publisher<std_msgs::msg::String>(name_, rclcpp::QoS(1).transient_local());

      std_msgs::msg::String msg;
      msg.data = content_;
      string_publisher_->publish(msg);
    }
    return content_;
  }

  // Load topic parameters.
  const auto keep_open_param = name_ + "_continuous";
  if (!node_->has_parameter(keep_open_param)) {
    node_->declare_parameter(keep_open_param, rclcpp::ParameterType::PARAMETER_BOOL);
  }
  bool keep_open;
  node_->get_parameter_or(keep_open_param, keep_open, default_continuous_value);

  const auto timeout_param = name_ + "_timeout";
  if (!node_->has_parameter(timeout_param)) {
    node_->declare_parameter(timeout_param, rclcpp::ParameterType::PARAMETER_DOUBLE);
  }
  double d_timeout;
  node_->get_parameter_or(timeout_param, d_timeout, default_timeout);  // ten second default
  const auto timeout = rclcpp::Duration::from_seconds(d_timeout);

  if (!waitForMessage(timeout)) {
    qCritical(
      "Could not find parameter %s and did not receive %s via std_msgs::msg::String subscription "
      "within %f seconds.",
      name_.c_str(),
      name_.c_str(),
      d_timeout);
  }
  if (!keep_open) {
    string_subscriber_.reset();
  }
  return content_;
}

bool SynchronizedStringParameter::getMainParameter()
{
  // Check if the parameter is declared, declare it if it's not declared yet.
  if (!node_->has_parameter(name_)) {
    node_->declare_parameter(name_, rclcpp::ParameterType::PARAMETER_STRING);
  }

  node_->get_parameter_or(name_, content_, std::string());

  return !content_.empty();
}

bool SynchronizedStringParameter::shouldPublish()
{
  const auto publish_param = "publish_" + name_;
  bool publish_string;
  if (!node_->has_parameter(publish_param)) {
    node_->declare_parameter(publish_param, rclcpp::ParameterType::PARAMETER_BOOL);
  }
  node_->get_parameter_or(publish_param, publish_string, false);
  return publish_string;
}

bool SynchronizedStringParameter::waitForMessage(const rclcpp::Duration& timeout)
{
  const auto nd_name = std::string(node_->get_name()).append("_ssp_").append(name_);
  rclcpp::Node temp_node(nd_name, node_->get_namespace());
  string_subscriber_ = temp_node.create_subscription<std_msgs::msg::String>(
    name_,
    rclcpp::QoS(1).transient_local().reliable(),  // "transient_local()" is required for supporting late subscriptions.
    [this](const std_msgs::msg::String::ConstSharedPtr& msg) { return stringCallback(msg); });

  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(string_subscriber_);

  const auto ret = wait_set.wait(timeout.to_chrono<ch::duration<double>>());
  if (ret.kind() == rclcpp::WaitResultKind::Ready) {
    std_msgs::msg::String msg;
    rclcpp::MessageInfo info;
    if (string_subscriber_->take(msg, info)) {
      content_ = msg.data;
      return true;
    }
  }
  return false;
}

void SynchronizedStringParameter::stringCallback(const std_msgs::msg::String::ConstSharedPtr& msg)
{
  if (msg->data == content_) {
    return;
  }
  if (parent_callback_) {
    parent_callback_(msg->data);
  }
  content_ = msg->data;
}
}  // namespace rviz
}  // namespace tobas

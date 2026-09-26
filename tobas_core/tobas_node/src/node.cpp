// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_node/node.hpp"

#include <tobas_ros2_tools/util.hpp>

namespace tobas
{
BaseNode::BaseNode(const std::string& node_name, const rclcpp::NodeOptions& options)
  : super(node_name, options), dparam_sub_(this)
{
  RCLCPP_INFO_STREAM(get_logger(), "Initializing '" << node_name << "'.");

  message_pub_ = createPublisher<tobas_msgs::msg::Message>(topic::kMessage, false, true, 1);
}

bool BaseNode::getBoolParam(const std::string& param_name)
{
  if (has_parameter(param_name)) {
    return get_parameter(param_name).as_bool();
  }
  else {
    return declareParam<bool>(param_name);
  }
}

long BaseNode::getIntParam(const std::string& param_name)
{
  if (has_parameter(param_name)) {
    return get_parameter(param_name).as_int();
  }
  else {
    return declareParam<long>(param_name);
  }
}

double BaseNode::getDoubleParam(const std::string& param_name)
{
  if (has_parameter(param_name)) {
    return get_parameter(param_name).as_double();
  }
  else {
    return declareParam<double>(param_name);
  }
}

std::string BaseNode::getStringParam(const std::string& param_name)
{
  if (has_parameter(param_name)) {
    return get_parameter(param_name).as_string();
  }
  else {
    return declareParam<std::string>(param_name);
  }
}

std::vector<bool> BaseNode::getBoolArrayParam(const std::string& param_name)
{
  if (has_parameter(param_name)) {
    return get_parameter(param_name).as_bool_array();
  }
  else {
    return declareParam<std::vector<bool>>(param_name);
  }
}

std::vector<uint8_t> BaseNode::getByteArrayParam(const std::string& param_name)
{
  if (has_parameter(param_name)) {
    return get_parameter(param_name).as_byte_array();
  }
  else {
    return declareParam<std::vector<uint8_t>>(param_name);
  }
}

std::vector<long> BaseNode::getIntArrayParam(const std::string& param_name)
{
  if (has_parameter(param_name)) {
    return get_parameter(param_name).as_integer_array();
  }
  else {
    return declareParam<std::vector<long>>(param_name);
  }
}

std::vector<double> BaseNode::getDoubleArrayParam(const std::string& param_name)
{
  if (has_parameter(param_name)) {
    return get_parameter(param_name).as_double_array();
  }
  else {
    return declareParam<std::vector<double>>(param_name);
  }
}

std::vector<std::string> BaseNode::getStringArrayParam(const std::string& param_name)
{
  if (has_parameter(param_name)) {
    return get_parameter(param_name).as_string_array();
  }
  else {
    return declareParam<std::vector<std::string>>(param_name);
  }
}

bool BaseNode::getBoolParam(const std::string& param_name, const bool& dflt) noexcept
{
  if (has_parameter(param_name)) {
    return get_parameter(param_name).as_bool();
  }
  else {
    return declareParam(param_name, dflt);
  }
}

long BaseNode::getIntParam(const std::string& param_name, const long& dflt) noexcept
{
  if (has_parameter(param_name)) {
    return get_parameter(param_name).as_int();
  }
  else {
    return declareParam(param_name, dflt);
  }
}

double BaseNode::getDoubleParam(const std::string& param_name, const double& dflt) noexcept
{
  if (has_parameter(param_name)) {
    return get_parameter(param_name).as_double();
  }
  else {
    return declareParam(param_name, dflt);
  }
}

std::string BaseNode::getStringParam(const std::string& param_name, const std::string& dflt) noexcept
{
  if (has_parameter(param_name)) {
    return get_parameter(param_name).as_string();
  }
  else {
    return declareParam(param_name, dflt);
  }
}

std::vector<bool> BaseNode::getBoolArrayParam(const std::string& param_name, const std::vector<bool>& dflt) noexcept
{
  if (has_parameter(param_name)) {
    return get_parameter(param_name).as_bool_array();
  }
  else {
    return declareParam(param_name, dflt);
  }
}

std::vector<uint8_t>
BaseNode::getByteArrayParam(const std::string& param_name, const std::vector<uint8_t>& dflt) noexcept
{
  if (has_parameter(param_name)) {
    return get_parameter(param_name).as_byte_array();
  }
  else {
    return declareParam(param_name, dflt);
  }
}

std::vector<long> BaseNode::getIntArrayParam(const std::string& param_name, const std::vector<long>& dflt) noexcept
{
  if (has_parameter(param_name)) {
    return get_parameter(param_name).as_integer_array();
  }
  else {
    return declareParam(param_name, dflt);
  }
}

std::vector<double>
BaseNode::getDoubleArrayParam(const std::string& param_name, const std::vector<double>& dflt) noexcept
{
  if (has_parameter(param_name)) {
    return get_parameter(param_name).as_double_array();
  }
  else {
    return declareParam(param_name, dflt);
  }
}

std::vector<std::string>
BaseNode::getStringArrayParam(const std::string& param_name, const std::vector<std::string>& dflt) noexcept
{
  if (has_parameter(param_name)) {
    return get_parameter(param_name).as_string_array();
  }
  else {
    return declareParam(param_name, dflt);
  }
}

rclcpp::NodeOptions BaseNode::nodeOptions_Default(rclcpp::NodeOptions options)
{
  options.enable_rosout(false);
  options.use_intra_process_comms(true);
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  setClockType(options);
  options.append_parameter_override("start_type_description_service", false);

  return options;
}

rclcpp::NodeOptions BaseNode::nodeOptions_DParam(rclcpp::NodeOptions options)
{
  return nodeOptions_Default(options).start_parameter_services(true).start_parameter_event_publisher(true);
}

void BaseNode::rclcppLog(uint8_t level, const std::string& text) const
{
  switch (level) {
    case tobas_msgs::msg::Message::LEVEL_DEBUG:
      RCLCPP_DEBUG_STREAM(get_logger(), text);
      break;
    case tobas_msgs::msg::Message::LEVEL_INFO:
      RCLCPP_INFO_STREAM(get_logger(), text);
      break;
    case tobas_msgs::msg::Message::LEVEL_WARN:
      RCLCPP_WARN_STREAM(get_logger(), text);
      break;
    case tobas_msgs::msg::Message::LEVEL_ERROR:
      RCLCPP_ERROR_STREAM(get_logger(), text);
      break;
    case tobas_msgs::msg::Message::LEVEL_FATAL:
      RCLCPP_FATAL_STREAM(get_logger(), text);
      break;
    default:
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid log level: " << static_cast<int>(level));
      break;
  }
}

void BaseNode::getDParamCb(
  const tobas_dparam_msgs::srv::GetParams::Request::ConstSharedPtr&,
  const tobas_dparam_msgs::srv::GetParams::Response::SharedPtr& res)
{
  res->params = dparams_;
}

void BaseNode::setClockType(rclcpp::NodeOptions& options)
{
  const auto clock_type = ros2::getEnv("TOBAS_CLOCK_TYPE");

  if (!clock_type) {
    return;
  }

  if (std::strcmp(clock_type, "ros_time") == 0) {
    options.clock_type(RCL_ROS_TIME);  // Use the system clock if no reference clock is available.
    options.use_clock_thread(true);    // Use a dedicated thread because `/clock` may be received.
  }
  else if (std::strcmp(clock_type, "system_time") == 0) {
    options.clock_type(RCL_SYSTEM_TIME);  // System clock synchronized with NTP.
    options.use_clock_thread(false);      // No dedicated thread is needed because `/clock` is not received.
  }
  else if (std::strcmp(clock_type, "steady_time") == 0) {
    options.clock_type(RCL_STEADY_TIME);  // Monotonic timer unaffected by NTP.
    options.use_clock_thread(false);      // No dedicated thread is needed because `/clock` is not received.
  }
  else {
    std::cerr << "Unknown clock type: " << clock_type << std::endl;
  }
}
}  // namespace tobas

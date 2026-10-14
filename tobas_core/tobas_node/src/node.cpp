// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_node/node.hpp"

#include <tobas_ros2_tools/util.hpp>

namespace tobas
{
BaseNode::BaseNode(const std::string& _node_name, const rclcpp::NodeOptions& _options)
  : super(_node_name, _options), dparam_sub_(this)
{
  RCLCPP_INFO_STREAM(get_logger(), "Initializing '" << _node_name << "'.");

  message_pub_ = createPublisher<tobas_msgs::msg::Message>(topic::kMessage, false, true, 1);
}

bool BaseNode::getBoolParam(const std::string& _param_name)
{
  if (has_parameter(_param_name)) {
    return get_parameter(_param_name).as_bool();
  }
  else {
    return declareParam<bool>(_param_name);
  }
}

long BaseNode::getIntParam(const std::string& _param_name)
{
  if (has_parameter(_param_name)) {
    return get_parameter(_param_name).as_int();
  }
  else {
    return declareParam<long>(_param_name);
  }
}

double BaseNode::getDoubleParam(const std::string& _param_name)
{
  if (has_parameter(_param_name)) {
    return get_parameter(_param_name).as_double();
  }
  else {
    return declareParam<double>(_param_name);
  }
}

std::string BaseNode::getStringParam(const std::string& _param_name)
{
  if (has_parameter(_param_name)) {
    return get_parameter(_param_name).as_string();
  }
  else {
    return declareParam<std::string>(_param_name);
  }
}

std::vector<bool> BaseNode::getBoolArrayParam(const std::string& _param_name)
{
  if (has_parameter(_param_name)) {
    return get_parameter(_param_name).as_bool_array();
  }
  else {
    return declareParam<std::vector<bool>>(_param_name);
  }
}

std::vector<uint8_t> BaseNode::getByteArrayParam(const std::string& _param_name)
{
  if (has_parameter(_param_name)) {
    return get_parameter(_param_name).as_byte_array();
  }
  else {
    return declareParam<std::vector<uint8_t>>(_param_name);
  }
}

std::vector<long> BaseNode::getIntArrayParam(const std::string& _param_name)
{
  if (has_parameter(_param_name)) {
    return get_parameter(_param_name).as_integer_array();
  }
  else {
    return declareParam<std::vector<long>>(_param_name);
  }
}

std::vector<double> BaseNode::getDoubleArrayParam(const std::string& _param_name)
{
  if (has_parameter(_param_name)) {
    return get_parameter(_param_name).as_double_array();
  }
  else {
    return declareParam<std::vector<double>>(_param_name);
  }
}

std::vector<std::string> BaseNode::getStringArrayParam(const std::string& _param_name)
{
  if (has_parameter(_param_name)) {
    return get_parameter(_param_name).as_string_array();
  }
  else {
    return declareParam<std::vector<std::string>>(_param_name);
  }
}

bool BaseNode::getBoolParam(const std::string& _param_name, bool _dflt) noexcept
{
  if (has_parameter(_param_name)) {
    return get_parameter(_param_name).as_bool();
  }
  else {
    return declareParam(_param_name, _dflt);
  }
}

long BaseNode::getIntParam(const std::string& _param_name, long _dflt) noexcept
{
  if (has_parameter(_param_name)) {
    return get_parameter(_param_name).as_int();
  }
  else {
    return declareParam(_param_name, _dflt);
  }
}

double BaseNode::getDoubleParam(const std::string& _param_name, double _dflt) noexcept
{
  if (has_parameter(_param_name)) {
    return get_parameter(_param_name).as_double();
  }
  else {
    return declareParam(_param_name, _dflt);
  }
}

std::string BaseNode::getStringParam(const std::string& _param_name, const std::string& _dflt) noexcept
{
  if (has_parameter(_param_name)) {
    return get_parameter(_param_name).as_string();
  }
  else {
    return declareParam(_param_name, _dflt);
  }
}

std::vector<bool> BaseNode::getBoolArrayParam(const std::string& _param_name, const std::vector<bool>& _dflt) noexcept
{
  if (has_parameter(_param_name)) {
    return get_parameter(_param_name).as_bool_array();
  }
  else {
    return declareParam(_param_name, _dflt);
  }
}

std::vector<uint8_t>
BaseNode::getByteArrayParam(const std::string& _param_name, const std::vector<uint8_t>& _dflt) noexcept
{
  if (has_parameter(_param_name)) {
    return get_parameter(_param_name).as_byte_array();
  }
  else {
    return declareParam(_param_name, _dflt);
  }
}

std::vector<long> BaseNode::getIntArrayParam(const std::string& _param_name, const std::vector<long>& _dflt) noexcept
{
  if (has_parameter(_param_name)) {
    return get_parameter(_param_name).as_integer_array();
  }
  else {
    return declareParam(_param_name, _dflt);
  }
}

std::vector<double>
BaseNode::getDoubleArrayParam(const std::string& _param_name, const std::vector<double>& _dflt) noexcept
{
  if (has_parameter(_param_name)) {
    return get_parameter(_param_name).as_double_array();
  }
  else {
    return declareParam(_param_name, _dflt);
  }
}

std::vector<std::string>
BaseNode::getStringArrayParam(const std::string& _param_name, const std::vector<std::string>& _dflt) noexcept
{
  if (has_parameter(_param_name)) {
    return get_parameter(_param_name).as_string_array();
  }
  else {
    return declareParam(_param_name, _dflt);
  }
}

rclcpp::NodeOptions BaseNode::nodeOptions_Default(rclcpp::NodeOptions _options)
{
  _options.enable_rosout(false);
  _options.use_intra_process_comms(true);
  _options.start_parameter_services(false);
  _options.start_parameter_event_publisher(false);
  setClockType(_options);
  _options.append_parameter_override("start_type_description_service", false);

  return _options;
}

rclcpp::NodeOptions BaseNode::nodeOptions_DParam(rclcpp::NodeOptions _options)
{
  return nodeOptions_Default(_options).start_parameter_services(true).start_parameter_event_publisher(true);
}

void BaseNode::rclcppLog(uint8_t _level, const std::string& _text) const
{
  switch (_level) {
    case tobas_msgs::msg::Message::LEVEL_DEBUG:
      RCLCPP_DEBUG_STREAM(get_logger(), _text);
      break;
    case tobas_msgs::msg::Message::LEVEL_INFO:
      RCLCPP_INFO_STREAM(get_logger(), _text);
      break;
    case tobas_msgs::msg::Message::LEVEL_WARN:
      RCLCPP_WARN_STREAM(get_logger(), _text);
      break;
    case tobas_msgs::msg::Message::LEVEL_ERROR:
      RCLCPP_ERROR_STREAM(get_logger(), _text);
      break;
    case tobas_msgs::msg::Message::LEVEL_FATAL:
      RCLCPP_FATAL_STREAM(get_logger(), _text);
      break;
    default:
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid log level: " << static_cast<int>(_level));
      break;
  }
}

void BaseNode::getDParamCb(
  const tobas_dparam_msgs::srv::GetParams::Request::ConstSharedPtr&,
  const tobas_dparam_msgs::srv::GetParams::Response::SharedPtr& _res)
{
  _res->params = dparams_;
}

void BaseNode::setClockType(rclcpp::NodeOptions& _options)
{
  const auto clock_type = ros2::getEnv("TOBAS_CLOCK_TYPE");

  if (!clock_type) {
    std::cerr << clock_type.error() << '\n';
    return;
  }

  if (std::strcmp(*clock_type, "ros_time") == 0) {
    _options.clock_type(RCL_ROS_TIME);  // Use the system clock if no reference clock is available.
    _options.use_clock_thread(true);    // Use a dedicated thread because `/clock` may be received.
  }
  else if (std::strcmp(*clock_type, "system_time") == 0) {
    _options.clock_type(RCL_SYSTEM_TIME);  // System clock synchronized with NTP.
    _options.use_clock_thread(false);      // No dedicated thread is needed because `/clock` is not received.
  }
  else if (std::strcmp(*clock_type, "steady_time") == 0) {
    _options.clock_type(RCL_STEADY_TIME);  // Monotonic timer unaffected by NTP.
    _options.use_clock_thread(false);      // No dedicated thread is needed because `/clock` is not received.
  }
  else {
    std::cerr << "Unknown clock type: " << *clock_type << '\n';
  }
}
}  // namespace tobas

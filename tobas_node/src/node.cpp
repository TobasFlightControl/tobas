#include "../include/tobas_node/node.hpp"

#include <rcutils/env.h>

#include <tobas_constants/constants.hpp>

using namespace std;

namespace tobas
{
BaseNode::BaseNode(const string& node_name, const rclcpp::NodeOptions& options)
  : super(node_name, createNodeOptions(options)), dparam_sub_(this)
{
  RCLCPP_INFO_STREAM(get_logger(), "Initializing \"" << node_name << "\".");

  message_pub_ = createPublisher<tobas_std_msgs::msg::Message>(kMessageTopic);
  get_dparam_ss_ = createService<tobas_dparam_msgs::srv::GetParams>(
    node_name + "/" + tobas::kGetDynamicParamsSrv, &self::getDParamCb, this);
}

bool BaseNode::getBoolParam(const string& name)
{
  if (has_parameter(name)) {
    return get_parameter(name).as_bool();
  }
  else {
    return declareParam<bool>(name);
  }
}

long BaseNode::getIntParam(const string& name)
{
  if (has_parameter(name)) {
    return get_parameter(name).as_int();
  }
  else {
    return declareParam<long>(name);
  }
}

double BaseNode::getDoubleParam(const string& name)
{
  if (has_parameter(name)) {
    return get_parameter(name).as_double();
  }
  else {
    return declareParam<double>(name);
  }
}

string BaseNode::getStringParam(const string& name)
{
  if (has_parameter(name)) {
    return get_parameter(name).as_string();
  }
  else {
    return declareParam<string>(name);
  }
}

vector<bool> BaseNode::getBoolArrayParam(const string& name)
{
  if (has_parameter(name)) {
    return get_parameter(name).as_bool_array();
  }
  else {
    return declareParam<vector<bool>>(name);
  }
}

vector<uint8_t> BaseNode::getByteArrayParam(const string& name)
{
  if (has_parameter(name)) {
    return get_parameter(name).as_byte_array();
  }
  else {
    return declareParam<vector<uint8_t>>(name);
  }
}

vector<long> BaseNode::getIntArrayParam(const string& name)
{
  if (has_parameter(name)) {
    return get_parameter(name).as_integer_array();
  }
  else {
    return declareParam<vector<long>>(name);
  }
}

vector<double> BaseNode::getDoubleArrayParam(const string& name)
{
  if (has_parameter(name)) {
    return get_parameter(name).as_double_array();
  }
  else {
    return declareParam<vector<double>>(name);
  }
}

vector<string> BaseNode::getStringArrayParam(const string& name)
{
  if (has_parameter(name)) {
    return get_parameter(name).as_string_array();
  }
  else {
    return declareParam<vector<string>>(name);
  }
}

bool BaseNode::getBoolParam(const string& name, const bool& _default)
{
  if (has_parameter(name)) {
    return get_parameter(name).as_bool();
  }
  else {
    return declareParam(name, _default);
  }
}

long BaseNode::getIntParam(const string& name, const long& _default)
{
  if (has_parameter(name)) {
    return get_parameter(name).as_int();
  }
  else {
    return declareParam(name, _default);
  }
}

double BaseNode::getDoubleParam(const string& name, const double& _default)
{
  if (has_parameter(name)) {
    return get_parameter(name).as_double();
  }
  else {
    return declareParam(name, _default);
  }
}

string BaseNode::getStringParam(const string& name, const string& _default)
{
  if (has_parameter(name)) {
    return get_parameter(name).as_string();
  }
  else {
    return declareParam(name, _default);
  }
}

vector<bool> BaseNode::getBoolArrayParam(const string& name, const vector<bool>& _default)
{
  if (has_parameter(name)) {
    return get_parameter(name).as_bool_array();
  }
  else {
    return declareParam(name, _default);
  }
}

vector<uint8_t> BaseNode::getByteArrayParam(const string& name, const vector<uint8_t>& _default)
{
  if (has_parameter(name)) {
    return get_parameter(name).as_byte_array();
  }
  else {
    return declareParam(name, _default);
  }
}

vector<long> BaseNode::getIntArrayParam(const string& name, const vector<long>& _default)
{
  if (has_parameter(name)) {
    return get_parameter(name).as_integer_array();
  }
  else {
    return declareParam(name, _default);
  }
}

vector<double> BaseNode::getDoubleArrayParam(const string& name, const vector<double>& _default)
{
  if (has_parameter(name)) {
    return get_parameter(name).as_double_array();
  }
  else {
    return declareParam(name, _default);
  }
}

vector<string> BaseNode::getStringArrayParam(const string& name, const vector<string>& _default)
{
  if (has_parameter(name)) {
    return get_parameter(name).as_string_array();
  }
  else {
    return declareParam(name, _default);
  }
}

void BaseNode::rclcppLog(uint8_t level, const string& text) const
{
  switch (level) {
    case tobas_std_msgs::msg::Message::LEVEL_DEBUG:
      RCLCPP_DEBUG_STREAM(get_logger(), text);
      break;
    case tobas_std_msgs::msg::Message::LEVEL_INFO:
      RCLCPP_INFO_STREAM(get_logger(), text);
      break;
    case tobas_std_msgs::msg::Message::LEVEL_WARN:
      RCLCPP_WARN_STREAM(get_logger(), text);
      break;
    case tobas_std_msgs::msg::Message::LEVEL_ERROR:
      RCLCPP_ERROR_STREAM(get_logger(), text);
      break;
    case tobas_std_msgs::msg::Message::LEVEL_FATAL:
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

string BaseNode::createID(const char* file, int line)
{
  return string(file) + ":" + to_string(line);
}

rclcpp::NodeOptions BaseNode::createNodeOptions(rclcpp::NodeOptions options)
{
  const char* clock_type = nullptr;
  const char* error = rcutils_get_env("TOBAS_CLOCK_TYPE", &clock_type);

  if (error) {
    cerr << "Failed to get clock type: " << error << endl;
    return options;
  }

  if (strlen(clock_type) == 0) {
    return options;
  }

  if (strcmp(clock_type, "ros_time") == 0) {
    return options.clock_type(RCL_ROS_TIME);  // 参照クロックがなければシステムクロック
  }
  else if (strcmp(clock_type, "system_time") == 0) {
    return options.clock_type(RCL_SYSTEM_TIME);  // NTPと同期したシステムクロック
  }
  else if (strcmp(clock_type, "steady_time") == 0) {
    return options.clock_type(RCL_STEADY_TIME);  // NTPの影響を受けないモノトニックタイマー
  }
  else {
    cerr << "Unknown clock type: " << clock_type << endl;
    return options;
  }
}
}  // namespace tobas

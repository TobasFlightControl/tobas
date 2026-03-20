#include "tobas_node/node.hpp"

#include <tobas_ros2_tools/util.hpp>

using namespace std;

namespace tobas
{
BaseNode::BaseNode(const string& node_name, const rclcpp::NodeOptions& options)
  : super(node_name, options), dparam_sub_(this)
{
  RCLCPP_INFO_STREAM(get_logger(), "Initializing \"" << node_name << "\".");

  message_pub_ = createPublisher<tobas_msgs::msg::Message>(topic::kMessage);
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

bool BaseNode::getBoolParam(const string& name, const bool& dflt) noexcept
{
  if (has_parameter(name)) {
    return get_parameter(name).as_bool();
  }
  else {
    return declareParam(name, dflt);
  }
}

long BaseNode::getIntParam(const string& name, const long& dflt) noexcept
{
  if (has_parameter(name)) {
    return get_parameter(name).as_int();
  }
  else {
    return declareParam(name, dflt);
  }
}

double BaseNode::getDoubleParam(const string& name, const double& dflt) noexcept
{
  if (has_parameter(name)) {
    return get_parameter(name).as_double();
  }
  else {
    return declareParam(name, dflt);
  }
}

string BaseNode::getStringParam(const string& name, const string& dflt) noexcept
{
  if (has_parameter(name)) {
    return get_parameter(name).as_string();
  }
  else {
    return declareParam(name, dflt);
  }
}

vector<bool> BaseNode::getBoolArrayParam(const string& name, const vector<bool>& dflt) noexcept
{
  if (has_parameter(name)) {
    return get_parameter(name).as_bool_array();
  }
  else {
    return declareParam(name, dflt);
  }
}

vector<uint8_t> BaseNode::getByteArrayParam(const string& name, const vector<uint8_t>& dflt) noexcept
{
  if (has_parameter(name)) {
    return get_parameter(name).as_byte_array();
  }
  else {
    return declareParam(name, dflt);
  }
}

vector<long> BaseNode::getIntArrayParam(const string& name, const vector<long>& dflt) noexcept
{
  if (has_parameter(name)) {
    return get_parameter(name).as_integer_array();
  }
  else {
    return declareParam(name, dflt);
  }
}

vector<double> BaseNode::getDoubleArrayParam(const string& name, const vector<double>& dflt) noexcept
{
  if (has_parameter(name)) {
    return get_parameter(name).as_double_array();
  }
  else {
    return declareParam(name, dflt);
  }
}

vector<string> BaseNode::getStringArrayParam(const string& name, const vector<string>& dflt) noexcept
{
  if (has_parameter(name)) {
    return get_parameter(name).as_string_array();
  }
  else {
    return declareParam(name, dflt);
  }
}

void BaseNode::setClockType(rclcpp::NodeOptions& options)
{
  const auto clock_type = ros2::getEnv("TOBAS_CLOCK_TYPE");

  if (!clock_type) {
    return;
  }

  if (strcmp(clock_type, "ros_time") == 0) {
    options.clock_type(RCL_ROS_TIME);  // 参照クロックがなければシステムクロック
    options.use_clock_thread(true);    // /clock を受信する可能性があるため専用スレッドを設ける
  }
  else if (strcmp(clock_type, "system_time") == 0) {
    options.clock_type(RCL_SYSTEM_TIME);  // NTPと同期したシステムクロック
    options.use_clock_thread(false);      // /clock を受信しないので専用スレッドは不要
  }
  else if (strcmp(clock_type, "steady_time") == 0) {
    options.clock_type(RCL_STEADY_TIME);  // NTPの影響を受けないモノトニックタイマー
    options.use_clock_thread(false);      // /clock を受信しないので専用スレッドは不要
  }
  else {
    cerr << "Unknown clock type: " << clock_type << endl;
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

void BaseNode::rclcppLog(uint8_t level, const string& text) const
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

string BaseNode::createID(const char* file, int line)
{
  return string(file) + ":" + to_string(line);
}
}  // namespace tobas

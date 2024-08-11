#include <tobas_constants/constants.hpp>

#include "../include/tobas_node/node.hpp"

using namespace std;

namespace tobas
{
BaseNode::BaseNode(const string& node_name, const rclcpp::NodeOptions& options)
  : super(node_name, options), dparam_sub_(this)
{
  message_pub_ = createPublisher<tobas_std_msgs::msg::Message>(kMessageTopic);
  dparams_pub_ = createPublisher<tobas_dparam_msgs::msg::Parameters>(node_name + "/" + kDynamicParamsTopic, true);

  TOBAS_INFO("Initializing \"", node_name, "\".");
}

void BaseNode::publishDynamicParameterDescriptions()
{
  auto dparams = std::make_unique<tobas_dparam_msgs::msg::Parameters>(dparams_);
  dparams_pub_->publish(move(dparams));
}

bool BaseNode::getBoolParam(const string& name)
{
  if (!has_parameter(name))
    declare_parameter<bool>(name);
  return get_parameter(name).as_bool();
}

long BaseNode::getIntParam(const string& name)
{
  if (!has_parameter(name))
    declare_parameter<long>(name);
  return get_parameter(name).as_int();
}

double BaseNode::getDoubleParam(const string& name)
{
  if (!has_parameter(name))
    declare_parameter<double>(name);
  return get_parameter(name).as_double();
}

string BaseNode::getStringParam(const string& name)
{
  if (!has_parameter(name))
    declare_parameter<string>(name);
  return get_parameter(name).as_string();
}

vector<bool> BaseNode::getBoolArrayParam(const string& name)
{
  if (!has_parameter(name))
    declare_parameter<vector<bool>>(name);
  return get_parameter(name).as_bool_array();
}

vector<uint8_t> BaseNode::getByteArrayParam(const string& name)
{
  if (!has_parameter(name))
    declare_parameter<vector<uint8_t>>(name);
  return get_parameter(name).as_byte_array();
}

vector<long> BaseNode::getIntArrayParam(const string& name)
{
  if (!has_parameter(name))
    declare_parameter<vector<long>>(name);
  return get_parameter(name).as_integer_array();
}

vector<double> BaseNode::getDoubleArrayParam(const string& name)
{
  if (!has_parameter(name))
    declare_parameter<vector<double>>(name);
  return get_parameter(name).as_double_array();
}

vector<string> BaseNode::getStringArrayParam(const string& name)
{
  if (!has_parameter(name))
    declare_parameter<vector<string>>(name);
  return get_parameter(name).as_string_array();
}

bool BaseNode::getBoolParam(const string& name, const bool& _default)
{
  if (!has_parameter(name))
    declare_parameter(name, _default);
  return get_parameter(name).as_bool();
}

long BaseNode::getIntParam(const string& name, const long& _default)
{
  if (!has_parameter(name))
    declare_parameter(name, _default);
  return get_parameter(name).as_int();
}

double BaseNode::getDoubleParam(const string& name, const double& _default)
{
  if (!has_parameter(name))
    declare_parameter(name, _default);
  return get_parameter(name).as_double();
}

string BaseNode::getStringParam(const string& name, const string& _default)
{
  if (!has_parameter(name))
    declare_parameter(name, _default);
  return get_parameter(name).as_string();
}

vector<bool> BaseNode::getBoolArrayParam(const string& name, const vector<bool>& _default)
{
  if (!has_parameter(name))
    declare_parameter(name, _default);
  return get_parameter(name).as_bool_array();
}

vector<uint8_t> BaseNode::getByteArrayParam(const string& name, const vector<uint8_t>& _default)
{
  if (!has_parameter(name))
    declare_parameter(name, _default);
  return get_parameter(name).as_byte_array();
}

vector<long> BaseNode::getIntArrayParam(const string& name, const vector<long>& _default)
{
  if (!has_parameter(name))
    declare_parameter(name, _default);
  return get_parameter(name).as_integer_array();
}

vector<double> BaseNode::getDoubleArrayParam(const string& name, const vector<double>& _default)
{
  if (!has_parameter(name))
    declare_parameter(name, _default);
  return get_parameter(name).as_double_array();
}

vector<string> BaseNode::getStringArrayParam(const string& name, const vector<string>& _default)
{
  if (!has_parameter(name))
    declare_parameter(name, _default);
  return get_parameter(name).as_string_array();
}

rclcpp::QoS BaseNode::makeQoS(bool latch, bool reliable, size_t queue_size)
{
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, queue_size));

  if (latch)
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  else
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  if (reliable)
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  else
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  return qos;
}

void BaseNode::rclcppLog(uint8_t level, const string& text) const
{
  switch (level)
  {
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

string BaseNode::createID(const char* file, int line)
{
  return string(file) + ":" + to_string(line);
}
}  // namespace tobas

#include <tobas_std_tools/string.hpp>

#include "../include/tobas_ros2_tools/node.hpp"

using namespace std;

namespace ros2
{
Node::Node(const string& node_name, const rclcpp::NodeOptions& options) : super(node_name, options)
{
  message_pub_ = create_publisher<tobas_std_msgs::msg::Message>(kMessageTopic, 1);
}

bool Node::getBoolParam(const string& name)
{
  if (!has_parameter(name))
    declare_parameter<bool>(name);
  return get_parameter(name).as_bool();
}

long Node::getIntParam(const string& name)
{
  if (!has_parameter(name))
    declare_parameter<long>(name);
  return get_parameter(name).as_int();
}

double Node::getDoubleParam(const string& name)
{
  if (!has_parameter(name))
    declare_parameter<double>(name);
  return get_parameter(name).as_double();
}

string Node::getStringParam(const string& name)
{
  if (!has_parameter(name))
    declare_parameter<string>(name);
  return get_parameter(name).as_string();
}

vector<bool> Node::getBoolArrayParam(const string& name)
{
  if (!has_parameter(name))
    declare_parameter<vector<bool>>(name);
  return get_parameter(name).as_bool_array();
}

vector<uint8_t> Node::getByteArrayParam(const string& name)
{
  if (!has_parameter(name))
    declare_parameter<vector<uint8_t>>(name);
  return get_parameter(name).as_byte_array();
}

vector<long> Node::getIntArrayParam(const string& name)
{
  if (!has_parameter(name))
    declare_parameter<vector<long>>(name);
  return get_parameter(name).as_integer_array();
}

vector<double> Node::getDoubleArrayParam(const string& name)
{
  if (!has_parameter(name))
    declare_parameter<vector<double>>(name);
  return get_parameter(name).as_double_array();
}

vector<string> Node::getStringArrayParam(const string& name)
{
  if (!has_parameter(name))
    declare_parameter<vector<string>>(name);
  return get_parameter(name).as_string_array();
}

bool Node::getBoolParam(const string& name, const bool& _default)
{
  if (!has_parameter(name))
    declareParam(name, _default);
  return get_parameter(name).as_bool();
}

long Node::getIntParam(const string& name, const long& _default)
{
  if (!has_parameter(name))
    declareParam(name, _default);
  return get_parameter(name).as_int();
}

double Node::getDoubleParam(const string& name, const double& _default)
{
  if (!has_parameter(name))
    declareParam(name, _default);
  return get_parameter(name).as_double();
}

string Node::getStringParam(const string& name, const string& _default)
{
  if (!has_parameter(name))
    declareParam(name, _default);
  return get_parameter(name).as_string();
}

vector<bool> Node::getBoolArrayParam(const string& name, const vector<bool>& _default)
{
  if (!has_parameter(name))
    declareParam(name, _default);
  return get_parameter(name).as_bool_array();
}

vector<uint8_t> Node::getByteArrayParam(const string& name, const vector<uint8_t>& _default)
{
  if (!has_parameter(name))
    declareParam(name, _default);
  return get_parameter(name).as_byte_array();
}

vector<long> Node::getIntArrayParam(const string& name, const vector<long>& _default)
{
  if (!has_parameter(name))
    declareParam(name, _default);
  return get_parameter(name).as_integer_array();
}

vector<double> Node::getDoubleArrayParam(const string& name, const vector<double>& _default)
{
  if (!has_parameter(name))
    declareParam(name, _default);
  return get_parameter(name).as_double_array();
}

vector<string> Node::getStringArrayParam(const string& name, const vector<string>& _default)
{
  if (!has_parameter(name))
    declareParam(name, _default);
  return get_parameter(name).as_string_array();
}
}  // namespace ros2

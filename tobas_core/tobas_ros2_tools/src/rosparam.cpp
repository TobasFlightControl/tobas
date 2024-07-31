#include <tobas_std_tools/string.hpp>

#include "../include/tobas_ros2_tools/rosparam.hpp"

using namespace std;
using namespace Eigen;

namespace ros2
{
void getParam(rclcpp::Node::SharedPtr node, const string& key, size_t& param)
{
  int tmp;
  getParam(node, key, tmp, NON_NEGATIVE);
  param = static_cast<size_t>(tmp);
}

void getParam(rclcpp::Node::SharedPtr node, const string& key, size_t& param, const size_t& _default)
{
  int tmp;
  getParam(node, key, tmp, static_cast<int>(_default));

  if (tmp < 0)
  {
    RCLCPP_ERROR_STREAM(
      node->get_logger(), "Negative value is specified for '" << key << "'. The default value is used.");
    param = _default;
    return;
  }

  param = static_cast<size_t>(tmp);
}

void getParam(rclcpp::Node::SharedPtr node, const string& key, uint8_t& param)
{
  int tmp;
  getParam(node, key, tmp);
  if (tmp < 0 || UINT8_MAX < tmp)
    throw runtime_error("The specified value for '" + key + "' is out of range of unsigned char.");
  param = static_cast<uint8_t>(tmp);
}

void getParam(rclcpp::Node::SharedPtr node, const string& key, uint8_t& param, const uint8_t& _default)
{
  int tmp;
  getParam(node, key, tmp, static_cast<int>(_default));

  if (tmp < 0 || UINT8_MAX < tmp)
  {
    RCLCPP_ERROR_STREAM(
      node->get_logger(), "The specified value for '" << key << "' is out of range of unsigned char. The default value "
                                                      << static_cast<int>(_default) << " is used.");
    param = _default;
    return;
  }

  param = static_cast<uint8_t>(tmp);
}

void getParam(rclcpp::Node::SharedPtr node, const string& key, Vector2d& param)
{
  vector<double> tmp;
  getParam(node, key, tmp);
  if (tmp.size() != 2)
    throw runtime_error("The size of '" + key + "' must be 2.");
  param = Map<Vector2d>(tmp.data());
}

void getParam(rclcpp::Node::SharedPtr node, const string& key, Vector2d& param, const Vector2d& _default)
{
  vector<double> param_vec;
  const vector<double> default_vec(_default.data(), _default.data() + _default.size());
  getParam(node, key, param_vec, default_vec);

  if (param_vec.size() != 2)
  {
    RCLCPP_ERROR_STREAM(
      node->get_logger(), "The size of specified vector for '" << key << "' is not 2. The default vector is used.");
    param = _default;
    return;
  }

  param = Map<Vector2d>(param_vec.data());
}

void getParam(rclcpp::Node::SharedPtr node, const string& key, Vector3d& param)
{
  vector<double> tmp;
  getParam(node, key, tmp);
  if (tmp.size() != 3)
    throw runtime_error("The size of '" + key + "' must be 3.");
  param = Map<Vector3d>(tmp.data());
}

void getParam(rclcpp::Node::SharedPtr node, const string& key, Vector3d& param, const Vector3d& _default)
{
  vector<double> param_vec;
  const vector<double> default_vec(_default.data(), _default.data() + _default.size());
  getParam(node, key, param_vec, default_vec);

  if (param_vec.size() != 3)
  {
    RCLCPP_ERROR_STREAM(
      node->get_logger(), "The size of specified vector for '" << key << "' is not 3. The default vector is used.");
    param = _default;
    return;
  }

  param = Map<Vector3d>(param_vec.data());
}

void getParam(rclcpp::Node::SharedPtr node, const string& key, VectorXd& param)
{
  vector<double> tmp;
  getParam(node, key, tmp);
  param = Map<VectorXd>(tmp.data(), tmp.size());
}
}  // namespace ros2

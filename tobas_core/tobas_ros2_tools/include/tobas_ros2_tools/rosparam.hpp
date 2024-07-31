#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Core>

#include <tobas_std_tools/vector.hpp>

namespace ros2
{
enum constraint_t
{
  NONE,
  POSITIVE,
  NEGATIVE,
  NON_NEGATIVE,
  NON_POSITIVE,
};

template <typename T>
void checkConstraint(const std::string& key, const T& param, const constraint_t& constraint)
{
  switch (constraint)
  {
    case NONE:
      break;
    case POSITIVE:
      if (param <= 0)
        throw std::runtime_error(key + " must be positive.");
      break;
    case NEGATIVE:
      if (param >= 0)
        throw std::runtime_error(key + " must be negative.");
      break;
    case NON_NEGATIVE:
      if (param < 0)
        throw std::runtime_error(key + " must be non-negative.");
      break;
    case NON_POSITIVE:
      if (param > 0)
        throw std::runtime_error(key + " must be non-positive.");
      break;
    default:
      throw std::runtime_error("Invalid constraint type: " + std::to_string(constraint));
  }
}

template <typename T>
void getParam(rclcpp::Node::SharedPtr node, const std::string& key, T& param)
{
  if (!node->get_parameter(key, param))
    throw std::runtime_error("Parameter '" + key + "' is not specified.");
}

template <typename T>
void getParam(rclcpp::Node::SharedPtr node, const std::string& key, std::pair<T, T>& param)
{
  std::vector<T> tmp;
  getParam(node, key, tmp);
  if (tmp.size() != 2)
    throw std::runtime_error("The size of '" + key + "' must be 2.");

  param.first = tmp[0];
  param.second = tmp[1];
}

template <typename T>
void getParam(rclcpp::Node::SharedPtr node, const std::string& key, T& param, const T& _default)
{
  if (!node->get_parameter(key, param))
  {
    param = _default;
    RCLCPP_WARN_STREAM(
      node->get_logger(), "Parameter '" << key << "' is not specified. The default '" << _default << "' is used.");
  }
}

template <typename T>
void getParam(rclcpp::Node::SharedPtr node, const std::string& key, T& param, const constraint_t& constraint)
{
  getParam(node, key, param);
  checkConstraint(key, param, constraint);
}

template <typename T>
void getParam(rclcpp::Node::SharedPtr node, const std::string& key, T& param, const T& _default, const constraint_t& constraint)
{
  getParam(node, key, param, _default);
  checkConstraint(key, param, constraint);
}

void getParam(rclcpp::Node::SharedPtr node, const std::string& key, size_t& param);
void getParam(rclcpp::Node::SharedPtr node, const std::string& key, size_t& param, const size_t& _default);
void getParam(rclcpp::Node::SharedPtr node, const std::string& key, uint8_t& param);
void getParam(rclcpp::Node::SharedPtr node, const std::string& key, uint8_t& param, const uint8_t& _default);

void getParam(rclcpp::Node::SharedPtr node, const std::string& key, Eigen::Vector2d& param);
void getParam(rclcpp::Node::SharedPtr node, const std::string& key, Eigen::Vector2d& param, const Eigen::Vector2d& _default);
void getParam(rclcpp::Node::SharedPtr node, const std::string& key, Eigen::Vector3d& param);
void getParam(rclcpp::Node::SharedPtr node, const std::string& key, Eigen::Vector3d& param, const Eigen::Vector3d& _default);

void getParam(rclcpp::Node::SharedPtr node, const std::string& key, Eigen::VectorXd& param);
}  // namespace ros2

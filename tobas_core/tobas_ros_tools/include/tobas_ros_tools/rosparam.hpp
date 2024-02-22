#pragma once

#include <ros/ros.h>
#include <string>
#include <Eigen/Core>

#include <tobas_std_tools/vector.hpp>

#include "./exception.hpp"

namespace tobas_ros
{
enum Constraint
{
  NONE,
  POSITIVE,
  NEGATIVE,
  NON_NEGATIVE,
  NON_POSITIVE,
};

template <typename T>
void checkConstraint(
  ros::NodeHandle& nh,
  const std::string& key,
  const T& param,
  const Constraint& constraint)
{
  switch (constraint)
  {
    case NONE:
      break;
    case POSITIVE:
      ROS_CHECK(nh, param > 0, key << " must be positive.");
      break;
    case NEGATIVE:
      ROS_CHECK(nh, param < 0, key << " must be negative.");
      break;
    case NON_NEGATIVE:
      ROS_CHECK(nh, param >= 0, key << " must be non-negative.");
      break;
    case NON_POSITIVE:
      ROS_CHECK(nh, param <= 0, key << " must be non-positive.");
      break;
    default:
      ROS_EXIT(nh, "Invalid constraint type.");
  }
}

template <typename T>
void getParam(ros::NodeHandle& nh, const std::string& key, T& param)
{
  ROS_CHECK(nh, nh.param(key, param, T()), "Parameter '" << key << "' is not specified.");
}

template <typename T>
void getParam(ros::NodeHandle& nh, const std::string& key, std::pair<T, T>& param)
{
  std::vector<T> tmp;
  getParam(nh, key, tmp);
  ROS_CHECK(nh, tmp.size() == 2, "The size of '" << key << "' must be 2.");

  param.first = tmp[0];
  param.second = tmp[1];
}

template <typename T>
void getParam(ros::NodeHandle& nh, const std::string& key, T& param, const T& _default)
{
  if (!nh.param(key, param, _default))
  {
    param = _default;
    ROS_WARN_STREAM(
      "Parameter '" << key << "' is not specified. The default '" << _default << "' is used.");
  }
}

template <typename T>
void getParam(ros::NodeHandle& nh, const std::string& key, T& param, const Constraint& constraint)
{
  getParam(nh, key, param);
  checkConstraint(nh, key, param, constraint);
}

template <typename T>
void getParam(
  ros::NodeHandle& nh,
  const std::string& key,
  T& param,
  const T& _default,
  const Constraint& constraint)
{
  getParam(nh, key, param, _default);
  checkConstraint(nh, key, param, constraint);
}

void getParam(ros::NodeHandle& nh, const std::string& key, size_t& param);
void getParam(ros::NodeHandle& nh, const std::string& key, size_t& param, const size_t& _default);
void getParam(ros::NodeHandle& nh, const std::string& key, uint8_t& param);
void getParam(ros::NodeHandle& nh, const std::string& key, uint8_t& param, const uint8_t& _default);

void getParam(ros::NodeHandle& nh, const std::string& key, Eigen::Vector3d& param);
void getParam(
  ros::NodeHandle& nh,
  const std::string& key,
  Eigen::Vector3d& param,
  const Eigen::Vector3d& _default);

/**
 * @brief keyにマッチするパラメータが存在する場合にtrueを返す．
 *
 * @note namespace直下のキーのみ．
 * @note keyに"/"が含まれてはならない．
 */
bool match(ros::NodeHandle& nh, const std::string& key);
}  // namespace tobas_ros

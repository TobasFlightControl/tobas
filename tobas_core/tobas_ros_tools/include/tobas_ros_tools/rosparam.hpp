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
void checkConstraint(const std::string& key, const T& param, const Constraint& constraint)
{
  switch (constraint)
  {
    case NONE:
      break;
    case POSITIVE:
      if (param <= 0)
      {
        ROS_THROW(key << " must be positive.");
      }
      break;
    case NEGATIVE:
      if (param >= 0)
      {
        ROS_THROW(key << " must be negative.");
      }
      break;
    case NON_NEGATIVE:
      if (param < 0)
      {
        ROS_THROW(key << " must be non-negative.");
      }
      break;
    case NON_POSITIVE:
      if (param > 0)
      {
        ROS_THROW(key << " must be non-positive.");
      }
      break;
    default:
      ROS_THROW("Invalid constraint type.");
  }
}

template <typename T>
void getParam(ros::NodeHandle& nh, const std::string& key, T& param)
{
  if (!nh.param(key, param, T()))
    ROS_THROW("Parameter '" << key << "' is not specified.");
}

template <typename T>
void getParam(ros::NodeHandle& nh, const std::string& key, std::pair<T, T>& param)
{
  std::vector<T> tmp;
  getParam(nh, key, tmp);
  if (tmp.size() != 2)
    ROS_THROW("The size of '" << key << "' must be 2.");

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
      "Parameter '" << key << "' is not specified. The default value '" << _default
                    << "' is used.");
  }
}

template <typename T>
void getParam(ros::NodeHandle& nh, const std::string& key, T& param, const Constraint& constraint)
{
  getParam(nh, key, param);
  checkConstraint(key, param, constraint);
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
  checkConstraint(key, param, constraint);
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

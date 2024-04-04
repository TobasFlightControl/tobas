#pragma once

#include <ros/ros.h>

namespace tobas
{
/**
 * @brief あらゆるノードの規定ノード．プログラミングの自由度を下げて見通しを良くすることが目的．
 */
class BaseNode
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  const std::string name_;

  explicit BaseNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const std::string& name);

  virtual void getRosParams() = 0;
  virtual void registerPublishers() = 0;
  virtual void registerSubscribers() = 0;

  inline std::string ns()
  {
    return nh_.getNamespace() + "/";
  }

  /* Alias for ros::TransportHints().reliable().tcpNoDelay(). */
  static ros::TransportHints tcpNoDelay(const bool& nodelay = true);
};
}  // namespace tobas

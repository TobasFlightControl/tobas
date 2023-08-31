#pragma once

#include <ros/ros.h>

#include <tobas_msgs/Event.h>

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

  ros::Publisher event_pub_;
  ros::Subscriber event_sub_;

  explicit BaseNode(ros::NodeHandle nh, ros::NodeHandle pnh);

  virtual void getRosParams() = 0;
  virtual void registerPublishers() = 0;
  virtual void registerSubscribers() = 0;

  virtual void eventCb(const tobas_msgs::Event& event) = 0;

  void requestShutdown();
};
}  // namespace tobas

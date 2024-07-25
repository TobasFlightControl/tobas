#pragma once

#include <ros/ros.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/ThrottleArray.h>

namespace tobas_latency_publisher
{
class LatencyPublisher : public tobas::BaseNode
{
  using self = LatencyPublisher;
  using super = tobas::BaseNode;

public:
  explicit LatencyPublisher(
    ros::NodeHandle& nh,
    ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  ros::Publisher latency_pub_;
  ros::Subscriber throttles_sub_;

  void throttlesCb(const tobas_msgs::ThrottleArrayConstPtr& msg);
};
}  // namespace tobas_latency_publisher

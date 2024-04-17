#pragma once

#include <ros/ros.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/RotorSpeeds.h>

namespace tobas_latency_publisher
{
class LatencyPublisher : public tobas::BaseNode
{
  using self = LatencyPublisher;
  using super = tobas::BaseNode;

public:
  explicit LatencyPublisher(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  ros::Publisher latency_pub_;
  ros::Subscriber tar_rot_speeds_sub_;

  void targetRotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& tar_speeds);
};
}  // namespace tobas_latency_publisher

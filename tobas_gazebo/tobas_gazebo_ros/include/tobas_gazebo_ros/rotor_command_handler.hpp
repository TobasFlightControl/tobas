#pragma once

#include <std_srvs/Trigger.h>

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/ThrottleArray.h>
#include <tobas_msgs/EnableRCOutput.h>

namespace tobas_gazebo_ros
{
class RotorCommandHandler : public tobas::BaseNode
{
  using self = RotorCommandHandler;
  using super = tobas::BaseNode;

public:
  explicit RotorCommandHandler(
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

private:
  tobas::Drone drone_;

  std::map<uint8_t, rclcpp::Publisher> throttle_pubs_;
  rclcpp::Subscriber throttles_sub_;
  rclcpp::ServiceServer enable_rcout_srv_;

  void throttlesCb(const tobas_msgs::ThrottleArrayConstPtr& throttles);
  bool enableRCOutputCb(tobas_msgs::EnableRCOutputRequest& req, tobas_msgs::EnableRCOutputResponse& res);
};
}  // namespace tobas_gazebo_ros

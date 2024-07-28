#pragma once

#include <tobas_tools/node.hpp>
#include <tobas_msgs/ThrottleArray.h>
#include <tobas_msgs/EnableRCOutput.h>

#include <tobas_a1_core/dshot.hpp>

namespace a1
{
class DShotDriver : public tobas::BaseNode
{
  using self = DShotDriver;
  using super = tobas::BaseNode;

public:
  explicit DShotDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name = ros::this_node::getName());

private:
  DShot dshot_;
  std::array<bool, DShot::kChannelSize> is_enabled_;

  ros::Subscriber throttles_sub_;
  ros::ServiceServer enable_rcout_srv_;

  void throttlesCb(const tobas_msgs::ThrottleArrayConstPtr& throttles);
  bool enableRCOutputCb(tobas_msgs::EnableRCOutputRequest& req, tobas_msgs::EnableRCOutputResponse& res);
};
}  // namespace a1

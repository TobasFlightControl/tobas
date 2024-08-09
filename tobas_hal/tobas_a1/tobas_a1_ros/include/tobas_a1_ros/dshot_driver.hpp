#pragma once

#include <tobas_node/node.hpp>
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
  explicit DShotDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  DShot dshot_;
  std::array<bool, DShot::kChannelSize> is_enabled_;

  SubscriberPtr<> throttles_sub_;
  ServicePtr<> enable_rcout_srv_;

  void throttlesCb(const tobas_msgs::ThrottleArray::ConstSharedPtr& throttles);
  bool enableRCOutputCb(tobas_msgs::EnableRCOutputRequest& req, tobas_msgs::EnableRCOutputResponse& res);
};
}  // namespace a1

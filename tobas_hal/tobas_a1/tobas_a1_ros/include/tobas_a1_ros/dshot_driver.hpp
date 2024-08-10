#pragma once

#include <tobas_node/node.hpp>
#include <tobas_msgs/msg/throttle_array.hpp>
#include <tobas_msgs/srv/enable_rc_output.hpp>

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

  void throttlesCb(const tobas_msgs::msg::ThrottleArray::ConstSharedPtr& throttles);
  bool enableRCOutputCb(tobas_msgs::srv::EnableRCOutput::Request& req, tobas_msgs::srv::EnableRCOutput::Response& res);
};
}  // namespace a1

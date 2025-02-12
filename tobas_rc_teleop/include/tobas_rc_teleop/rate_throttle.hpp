#pragma once

#include <tobas_command_msgs/msg/rate_throttle.hpp>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class RateThrottleController : public BaseController
{
  using super = BaseController;

public:
  explicit RateThrottleController();

  void initialize(tobas::BaseNode* node) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void update(const tobas_msgs::msg::RCInput& rcin, const tobas_msgs::Odometry& odom) override;

private:
  // rosparams
  double max_attitude_rate_;  // [rad/s]
  double max_heading_rate_;   // [rad/s]

  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::msg::RateThrottle> cmd_pub_;

  void getStaticRosParams(tobas::BaseNode* node);
};
}  // namespace tobas_rc_teleop

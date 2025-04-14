#pragma once

#include <tobas_command_msgs_adapter/rate_throttle.hpp>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class RateThrottleController : public BaseController
{
  using self = RateThrottleController;
  using super = BaseController;

public:
  explicit RateThrottleController();

  bool requirePosition() override;
  bool requireOrientation() override;
  bool requireLinearVelocity() override;
  bool requireAngularVelocity() override;

  void initialize(tobas::BaseNode* node, tobas::flight_mode_t mode) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom) override;

private:
  // rosparams
  double max_atti_rate_;  // [rad/s]
  double max_head_rate_;  // [rad/s]

  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::RateThrottle> cmd_pub_;

  bool maxAttitudeRateCb(const double& p);
  bool maxHeadingRateCb(const double& p);
};
}  // namespace tobas_rc_teleop

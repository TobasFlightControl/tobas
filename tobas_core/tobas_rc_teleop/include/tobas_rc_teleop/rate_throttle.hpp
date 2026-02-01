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
  bool requireVelocity() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(tobas::BaseNode* node, tobas::FlightMode mode) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom) override;

private:
  // rosparams
  double max_atti_rate_;  // [rad/s]
  double max_head_rate_;  // [rad/s]
  double atti_expo_;
  double head_expo_;
  double throt_expo_;

  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::RateThrottle> cmd_pub_;

  bool maxAttitudeRateCb(const long& p);
  bool maxHeadingRateCb(const long& p);
  bool attitudeExpoCb(const long& p);
  bool headingExpoCb(const long& p);
  bool throttleExpoCb(const long& p);
};
}  // namespace tobas_rc_teleop

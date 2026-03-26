#pragma once

#include <tobas_command_msgs_adapter/rate_throttle_vector.hpp>

#include "./base_controller.hpp"

namespace tobas
{
namespace rc
{
class RateThrottleVectorController : public BaseController
{
  using self = RateThrottleVectorController;
  using super = BaseController;

public:
  explicit RateThrottleVectorController();

  bool requirePosition() override;
  bool requireVelocity() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(tobas::BaseNode* node, tobas::FlightMode mode) override;
  void reset(const builtin_interfaces::msg::Time& stamp, const tobas_msgs::Odometry& setpoint, bool landed) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed) override;

private:
  // rosparams
  double max_atti_rate_;     // [rad/s]
  double max_head_rate_;     // [rad/s]
  double max_thrust_angle_;  // [rad]
  double atti_expo_;
  double head_expo_;
  double throt_expo_;

  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::RateThrottleVector> cmd_pub_;

  bool maxAttitudeRateCb(const double& p);
  bool maxHeadingRateCb(const double& p);
  bool maxThrustAngleCb(const double& p);
  bool attitudeExpoCb(const double& p);
  bool headingExpoCb(const double& p);
  bool throttleExpoCb(const double& p);
};
}  // namespace rc
}  // namespace tobas

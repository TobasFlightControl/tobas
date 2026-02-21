#pragma once

#include <tobas_command_msgs_adapter/angle_throttle_vector.hpp>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class AngleThrottleVectorController : public BaseController
{
  using self = AngleThrottleVectorController;
  using super = BaseController;

public:
  explicit AngleThrottleVectorController();

  bool requirePosition() override;
  bool requireVelocity() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(tobas::BaseNode* node, tobas::FlightMode mode) override;
  void reset(const tobas_msgs::Odometry& odom) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom) override;

private:
  double yaw_;
  rclcpp::Time t_last_rcin_;

  // rosparams
  double max_attitude_;      // [rad]
  double max_head_rate_;     // [rad/s]
  double max_thrust_angle_;  // [rad]
  double atti_expo_;
  double head_expo_;
  double throt_expo_;

  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::AngleThrottleVector> cmd_pub_;

  bool maxAttitudeCb(const long& p);
  bool maxHeadingRateCb(const long& p);
  bool maxThrustAngleCb(const long& p);
  bool attitudeExpoCb(const long& p);
  bool headingExpoCb(const long& p);
  bool throttleExpoCb(const long& p);
};
}  // namespace tobas_rc_teleop

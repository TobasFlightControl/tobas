#pragma once

#include <tobas_control/online_trajectory_generation/velocity_limited.hpp>

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
  void reset(const tobas_msgs::Odometry& odom, bool landed) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed) override;

private:
  rclcpp::Time t_last_rcin_;
  ctrl::VelocityLimitedOnlineTrajectoryGenerator roll_filt_, pitch_filt_, thrust_angle_filt_;
  double tar_yaw_;

  // rosparams
  double max_roll_;          // [rad]
  double max_pitch_;         // [rad]
  double max_yaw_rate_;      // [rad/s]
  double max_thrust_angle_;  // [rad]
  double roll_expo_;
  double yaw_expo_;
  double throt_expo_;

  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::AngleThrottleVector> cmd_pub_;

  bool maxRollCb(const long& p);
  bool maxRollRateCb(const long& p);
  bool maxPitchCb(const long& p);
  bool maxPitchRateCb(const long& p);
  bool maxYawRateCb(const long& p);
  bool maxThrustAngleCb(const long& p);
  bool maxThrustAngleRateCb(const long& p);
  bool rollExpoCb(const long& p);
  bool yawExpoCb(const long& p);
  bool throttleExpoCb(const long& p);
};
}  // namespace tobas_rc_teleop

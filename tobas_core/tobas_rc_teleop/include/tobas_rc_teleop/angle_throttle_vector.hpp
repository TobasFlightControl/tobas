#pragma once

#include <tobas_trajectory_generation/online/velocity_limited.hpp>

#include <tobas_command_msgs_adapter/angle_throttle_vector.hpp>

#include "./base_controller.hpp"

namespace tobas
{
namespace rc
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

  void initialize(BaseNode* node, FlightMode mode) override;
  void reset(const builtin_interfaces::msg::Time& stamp, const tobas_msgs::Odometry& setpoint, bool landed) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed) override;

private:
  rclcpp::Time t_last_rcin_;
  traj::VelocityLimitedOnlineTrajectoryGenerator roll_filt_, pitch_filt_, thrust_angle_filt_;
  double tar_yaw_;

  // rosparams
  double max_roll_;          // [rad]
  double max_pitch_;         // [rad]
  double max_yaw_rate_;      // [rad/s]
  double max_thrust_angle_;  // [rad]
  double roll_expo_;
  double yaw_expo_;
  double throt_expo_;
  double thrust_angle_expo_;

  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::AngleThrottleVector> cmd_pub_;

  bool maxRollCb(const double& p);
  bool maxRollRateCb(const double& p);
  bool maxPitchCb(const double& p);
  bool maxPitchRateCb(const double& p);
  bool maxYawRateCb(const double& p);
  bool maxThrustAngleCb(const double& p);
  bool maxThrustAngleRateCb(const double& p);
  bool rollExpoCb(const double& p);
  bool yawExpoCb(const double& p);
  bool throttleExpoCb(const double& p);
  bool thrustAngleExpoCb(const double& p);
};
}  // namespace rc
}  // namespace tobas

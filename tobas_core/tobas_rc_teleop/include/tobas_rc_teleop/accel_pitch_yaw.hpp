#pragma once

#include <tobas_control/online_trajectory_generation/velocity_limited.hpp>

#include <tobas_command_msgs_adapter/accel_pitch_yaw.hpp>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class AccelPitchYawController : public BaseController
{
  using self = AccelPitchYawController;
  using super = BaseController;

public:
  explicit AccelPitchYawController();

  bool requirePosition() override;
  bool requireVelocity() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(tobas::BaseNode* node, tobas::FlightMode mode) override;
  void reset(const tobas_msgs::Odometry& odom, bool landed) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed) override;

private:
  rclcpp::Time t_last_rcin_;
  ctrl::VelocityLimitedOnlineTrajectoryGenerator ax_filt_, ay_filt_, pitch_filt_;
  double tar_yaw_;

  // rosparams
  double max_hor_acc_;   // [m/s]
  double max_ver_acc_;   // [m/s]
  double max_pitch_;     // [rad]
  double max_yaw_rate_;  // [rad/s]
  double hor_acc_expo_;
  double ver_acc_expo_;
  double pitch_expo_;
  double yaw_expo_;

  // Publisher
  ros2::PublisherPtr<tobas_command_msgs::AccelPitchYaw> cmd_pub_;

  bool maxHorizontalAccelCb(const double& p);
  bool maxHorizontalJerkCb(const double& p);
  bool maxVerticalAccelCb(const double& p);
  bool maxPitchCb(const long& p);
  bool maxPitchRateCb(const long& p);
  bool maxYawRateCb(const long& p);
  bool horizontalAccelExpoCb(const long& p);
  bool verticalAccelExpoCb(const long& p);
  bool pitchExpoCb(const long& p);
  bool yawExpoCb(const long& p);
};
}  // namespace tobas_rc_teleop

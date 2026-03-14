#pragma once

#include <tobas_trajectory_generation/online/velocity_limited.hpp>

#include <tobas_command_msgs_adapter/pos_vel_acc_yaw.hpp>

#include "./base_controller.hpp"

namespace tobas_rc_teleop
{
class PosVelAccYawController : public BaseController
{
  using self = PosVelAccYawController;
  using super = BaseController;

public:
  explicit PosVelAccYawController();

  bool requirePosition() override;
  bool requireVelocity() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(tobas::BaseNode* node, tobas::FlightMode mode) override;
  void reset(const tobas_msgs::Odometry& odom, bool landed) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed) override;

private:
  builtin_interfaces::msg::Time t_last_rcin_;
  ctrl::VelocityLimitedOnlineTrajectoryGenerator vx_filt_, vy_filt_;
  kdl::Vector tar_pos_W_;
  double tar_yaw_;

  // rosparams
  double max_hor_vel_;    // [m/s]
  double max_ver_vel_;    // [m/s]
  double max_head_rate_;  // [rad/s]
  double max_ep_down_;    // [m]
  double hor_vel_expo_;
  double ver_vel_expo_;
  double head_expo_;

  // Publisher
  ros2::PublisherPtr<tobas_command_msgs::PosVelAccYaw> cmd_pub_;

  bool maxHorizontalVelocityCb(const double& p);
  bool maxHorizontalAccelCb(const double& p);
  bool maxVerticalVelocityCb(const double& p);
  bool maxHeadingRateCb(const double& p);
  bool maxPositionErrorDown(const double& p);
  bool horizontalVelocityExpoCb(const double& p);
  bool verticalVelocityExpoCb(const double& p);
  bool headingExpoCb(const double& p);
};
}  // namespace tobas_rc_teleop

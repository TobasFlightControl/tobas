// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_trajectory_generation/online/velocity_limited.hpp>

#include <tobas_command_msgs_adapter/pos_vel_acc_pitch_yaw.hpp>

#include "./base_controller.hpp"
#include "./filter/second_order_velocity_filter.hpp"

namespace tobas::rc
{
class PosVelAccPitchYawController : public BaseController
{
  using self = PosVelAccPitchYawController;

public:
  explicit PosVelAccPitchYawController();

  bool requireHorizontalPosition() override;
  bool requireVerticalPosition() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(BaseNode* _node, FlightMode _mode) override;
  void reset(const builtin_interfaces::msg::Time& _stamp, const tobas_msgs::Odometry& _setpoint, bool _landed) override;
  void update(const tobas_msgs::RCInput& _rcin, const tobas_msgs::Odometry& _odom, bool _landed) override;

private:
  rclcpp::Time t_last_rcin_;
  SecondOrderVelocityFilter vx_filt_, vy_filt_, vz_filt_;
  traj::VelocityLimitedOnlineTrajectoryGenerator pitch_filt_;
  kdl::Vector tar_pos_W_;
  double tar_yaw_;

  // ROS parameters
  double max_hor_vel_;   ///< [m/s]
  double max_ver_vel_;   ///< [m/s]
  double max_pitch_;     ///< [rad]
  double max_yaw_rate_;  ///< [rad/s]
  double max_ep_down_;   ///< [m]
  double hor_vel_expo_;
  double ver_vel_expo_;
  double pitch_expo_;
  double yaw_expo_;

  // Publisher
  ros2::PublisherPtr<tobas_command_msgs::PosVelAccPitchYaw> cmd_pub_;

  void maxHorizontalVelocityCb(double _p);
  void maxHorizontalJerkCb(double _p);
  void maxVerticalVelocityCb(double _p);
  void maxVerticalJerkCb(double _p);
  void maxPitchCb(double _p);
  void maxPitchRateCb(double _p);
  void maxYawRateCb(double _p);
  void maxPositionErrorDown(double _p);
  void horizontalVelocityExpoCb(double _p);
  void verticalVelocityExpoCb(double _p);
  void pitchExpoCb(double _p);
  void yawExpoCb(double _p);
};
}  // namespace tobas::rc

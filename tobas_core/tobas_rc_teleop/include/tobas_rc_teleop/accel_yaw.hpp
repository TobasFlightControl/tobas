// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_trajectory_generation/online/velocity_limited.hpp>

#include <tobas_command_msgs_adapter/accel_yaw.hpp>

#include "./base_controller.hpp"

namespace tobas
{
namespace rc
{
class AccelYawController : public BaseController
{
  using self = AccelYawController;
  using super = BaseController;

public:
  explicit AccelYawController();

  bool requireHorizontalPosition() override;
  bool requireVerticalPosition() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(BaseNode* node, FlightMode mode) override;
  void reset(const builtin_interfaces::msg::Time& stamp, const tobas_msgs::Odometry& setpoint, bool landed) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed) override;

private:
  builtin_interfaces::msg::Time t_last_rcin_;
  traj::VelocityLimitedOnlineTrajectoryGenerator ax_filt_, ay_filt_;
  double tar_yaw_;

  // ROS parameters.
  double max_hor_acc_;    // [m/s]
  double max_ver_acc_;    // [m/s]
  double max_head_rate_;  // [rad/s]
  double hor_acc_expo_;
  double ver_acc_expo_;
  double head_expo_;

  // Publisher
  ros2::PublisherPtr<tobas_command_msgs::AccelYaw> cmd_pub_;

  bool maxHorizontalAccelCb(const double& p);
  bool maxHorizontalJerkCb(const double& p);
  bool maxVerticalAccelCb(const double& p);
  bool maxHeadingRateCb(const double& p);
  bool horizontalAccelExpoCb(const double& p);
  bool verticalAccelExpoCb(const double& p);
  bool headingExpoCb(const double& p);
};
}  // namespace rc
}  // namespace tobas

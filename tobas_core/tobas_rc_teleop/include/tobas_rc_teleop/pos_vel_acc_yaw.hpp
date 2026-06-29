// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_command_msgs_adapter/pos_vel_acc_yaw.hpp>

#include "./base_controller.hpp"
#include "./filter/second_order_velocity_filter.hpp"

namespace tobas
{
namespace rc
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

  void initialize(BaseNode* node, FlightMode mode) override;
  void reset(const builtin_interfaces::msg::Time& stamp, const tobas_msgs::Odometry& setpoint, bool landed) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed) override;

private:
  builtin_interfaces::msg::Time t_last_rcin_;
  SecondOrderVelocityFilter vx_filt_, vy_filt_, vz_filt_;
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
  bool maxHorizontalJerkCb(const double& p);
  bool maxVerticalVelocityCb(const double& p);
  bool maxVerticalJerkCb(const double& p);
  bool maxHeadingRateCb(const double& p);
  bool maxPositionErrorDown(const double& p);
  bool horizontalVelocityExpoCb(const double& p);
  bool verticalVelocityExpoCb(const double& p);
  bool headingExpoCb(const double& p);
};
}  // namespace rc
}  // namespace tobas

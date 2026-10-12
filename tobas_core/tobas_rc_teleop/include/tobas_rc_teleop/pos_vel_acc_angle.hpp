// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_trajectory_generation/online/velocity_limited.hpp>

#include <tobas_command_msgs_adapter/angle.hpp>
#include <tobas_command_msgs_adapter/pos_vel_acc.hpp>

#include "./base_controller.hpp"
#include "./filter/second_order_velocity_filter.hpp"

namespace tobas
{
namespace rc
{
class PosVelAccAngleController : public BaseController
{
  using self = PosVelAccAngleController;

public:
  explicit PosVelAccAngleController();

  bool requireHorizontalPosition() override;
  bool requireVerticalPosition() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(BaseNode* node, FlightMode mode) override;
  void reset(const builtin_interfaces::msg::Time& stamp, const tobas_msgs::Odometry& setpoint, bool landed) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed) override;

private:
  rclcpp::Time t_last_rcin_;
  SecondOrderVelocityFilter vx_filt_, vy_filt_, vz_filt_;
  traj::VelocityLimitedOnlineTrajectoryGenerator roll_filt_, pitch_filt_;
  kdl::Vector tar_pos_W_;
  double tar_yaw_;

  // ROS parameters
  double max_hor_vel_;    ///< [m/s]
  double max_ver_vel_;    ///< [m/s]
  double max_attitude_;   ///< [rad]
  double max_head_rate_;  ///< [rad/s]
  double max_ep_down_;    ///< [m]
  double hor_vel_expo_;
  double ver_vel_expo_;
  double atti_expo_;
  double head_expo_;

  // Publisher
  ros2::PublisherPtr<tobas_command_msgs::PosVelAcc> pos_vel_acc_pub_;
  ros2::PublisherPtr<tobas_command_msgs::Angle> angle_pub_;

  void publishPosVelAcc(
    const builtin_interfaces::msg::Time& stamp,
    const kdl::Vector& pos,
    const kdl::Vector& vel,
    const kdl::Vector& acc);
  void publishAngle(const builtin_interfaces::msg::Time& stamp, double roll, double pitch, double yaw);

  void maxHorizontalVelocityCb(double p);
  void maxHorizontalJerkCb(double p);
  void maxVerticalVelocityCb(double p);
  void maxVerticalJerkCb(double p);
  void maxAttitudeCb(double p);
  void maxAttitudeRateCb(double p);
  void maxHeadingRateCb(double p);
  void maxPositionErrorDown(double p);
  void horizontalVelocityExpoCb(double p);
  void verticalVelocityExpoCb(double p);
  void attitudeExpoCb(double p);
  void headingExpoCb(double p);
};
}  // namespace rc
}  // namespace tobas

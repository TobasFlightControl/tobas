// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_trajectory_generation/online/velocity_limited.hpp>

#include <tobas_command_msgs_adapter/accel.hpp>
#include <tobas_command_msgs_adapter/angle.hpp>

#include "./base_controller.hpp"

namespace tobas
{
namespace rc
{
class AccelAngleController : public BaseController
{
  using self = AccelAngleController;

public:
  explicit AccelAngleController();

  bool requireHorizontalPosition() override;
  bool requireVerticalPosition() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(BaseNode* node, FlightMode mode) override;
  void reset(const builtin_interfaces::msg::Time& stamp, const tobas_msgs::Odometry& setpoint, bool landed) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed) override;

private:
  rclcpp::Time t_last_rcin_;
  traj::VelocityLimitedOnlineTrajectoryGenerator ax_filt_, ay_filt_, roll_filt_, pitch_filt_;
  double tar_yaw_;

  // ROS parameters
  double max_hor_acc_;    ///< [m/s]
  double max_ver_acc_;    ///< [m/s]
  double max_attitude_;   ///< [rad]
  double max_head_rate_;  ///< [rad/s]
  double hor_acc_expo_;
  double ver_acc_expo_;
  double atti_expo_;
  double head_expo_;

  // Publisher
  ros2::PublisherPtr<tobas_command_msgs::Accel> accel_pub_;
  ros2::PublisherPtr<tobas_command_msgs::Angle> angle_pub_;

  void publishAccel(const builtin_interfaces::msg::Time& stamp, const kdl::Vector& acc);
  void publishAngle(const builtin_interfaces::msg::Time& stamp, double roll, double pitch, double yaw);

  void maxHorizontalAccelCb(double p);
  void maxHorizontalJerkCb(double p);
  void maxVerticalAccelCb(double p);
  void maxAttitudeCb(double p);
  void maxAttitudeRateCb(double p);
  void maxHeadingRateCb(double p);
  void horizontalAccelExpoCb(double p);
  void verticalAccelExpoCb(double p);
  void attitudeExpoCb(double p);
  void headingExpoCb(double p);
};
}  // namespace rc
}  // namespace tobas

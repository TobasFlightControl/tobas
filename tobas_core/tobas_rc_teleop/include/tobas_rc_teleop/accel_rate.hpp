// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_command_msgs_adapter/accel.hpp>
#include <tobas_command_msgs_adapter/rate.hpp>

#include "./base_controller.hpp"

namespace tobas
{
namespace rc
{
class AccelRateController : public BaseController
{
  using self = AccelRateController;

public:
  explicit AccelRateController();

  bool requireHorizontalPosition() override;
  bool requireVerticalPosition() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(BaseNode* _node, FlightMode _mode) override;
  void reset(const builtin_interfaces::msg::Time& _stamp, const tobas_msgs::Odometry& _setpoint, bool _landed) override;
  void update(const tobas_msgs::RCInput& _rcin, const tobas_msgs::Odometry& _odom, bool _landed) override;

private:
  kdl::Vector tar_acc_G_;
  kdl::Vector tar_gyro_B_;

  // ROS parameters
  double max_hor_acc_;    ///< [m/s]
  double max_ver_acc_;    ///< [m/s]
  double max_atti_rate_;  ///< [rad/s]
  double max_head_rate_;  ///< [rad/s]
  double hor_acc_expo_;
  double ver_acc_expo_;
  double atti_expo_;
  double head_expo_;

  // Publisher
  ros2::PublisherPtr<tobas_command_msgs::Accel> accel_pub_;
  ros2::PublisherPtr<tobas_command_msgs::Rate> rate_pub_;

  void publishAccel(const builtin_interfaces::msg::Time& _stamp, const kdl::Vector& _acc);
  void publishRate(const builtin_interfaces::msg::Time& _stamp, const kdl::Vector& _rate);

  void maxHorizontalAccelCb(double _p);
  void maxVerticalAccelCb(double _p);
  void maxAttitudeRateCb(double _p);
  void maxHeadingRateCb(double _p);
  void horizontalAccelExpoCb(double _p);
  void verticalAccelExpoCb(double _p);
  void attitudeExpoCb(double _p);
  void headingExpoCb(double _p);
};
}  // namespace rc
}  // namespace tobas

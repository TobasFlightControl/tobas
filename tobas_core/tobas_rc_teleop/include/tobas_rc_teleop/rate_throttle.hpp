// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_command_msgs_adapter/rate_throttle.hpp>

#include "./base_controller.hpp"

namespace tobas
{
namespace rc
{
class RateThrottleController : public BaseController
{
  using self = RateThrottleController;

public:
  explicit RateThrottleController();

  bool requireHorizontalPosition() override;
  bool requireVerticalPosition() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(BaseNode* _node, FlightMode _mode) override;
  void reset(const builtin_interfaces::msg::Time& _stamp, const tobas_msgs::Odometry& _setpoint, bool _landed) override;
  void update(const tobas_msgs::RCInput& _rcin, const tobas_msgs::Odometry& _odom, bool _landed) override;

private:
  // ROS parameters
  double max_atti_rate_;  ///< [rad/s]
  double max_head_rate_;  ///< [rad/s]
  double atti_expo_;
  double head_expo_;
  double throt_expo_;

  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::RateThrottle> cmd_pub_;

  void maxAttitudeRateCb(double _p);
  void maxHeadingRateCb(double _p);
  void attitudeExpoCb(double _p);
  void headingExpoCb(double _p);
  void throttleExpoCb(double _p);
};
}  // namespace rc
}  // namespace tobas

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
  using super = BaseController;

public:
  explicit RateThrottleController();

  bool requirePosition() override;
  bool requireVelocity() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(BaseNode* node, FlightMode mode) override;
  void reset(const builtin_interfaces::msg::Time& stamp, const tobas_msgs::Odometry& setpoint, bool landed) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed) override;

private:
  // ROS parameters.
  double max_atti_rate_;  // [rad/s]
  double max_head_rate_;  // [rad/s]
  double atti_expo_;
  double head_expo_;
  double throt_expo_;

  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::RateThrottle> cmd_pub_;

  bool maxAttitudeRateCb(const double& p);
  bool maxHeadingRateCb(const double& p);
  bool attitudeExpoCb(const double& p);
  bool headingExpoCb(const double& p);
  bool throttleExpoCb(const double& p);
};
}  // namespace rc
}  // namespace tobas

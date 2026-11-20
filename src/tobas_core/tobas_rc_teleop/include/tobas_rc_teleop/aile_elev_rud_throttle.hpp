// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_command_msgs/msg/aile_elev_rud_throttle.hpp>

#include "./base_controller.hpp"

namespace tobas::rc
{
class AileElevRudThrottleController : public BaseController
{
  using self = AileElevRudThrottleController;
  using super = BaseController;

  static constexpr double kMaxDeflectionCommand = 1.0;
  static constexpr double kMinDeflectionCommand = -1.0;

public:
  explicit AileElevRudThrottleController();

  bool requireHorizontalPosition() const override;
  bool requireVerticalPosition() const override;
  bool requireAttitude() const override;
  bool requireHeading() const override;

  void initialize(BaseNode* _node, FlightMode _mode) override;
  void reset(const builtin_interfaces::msg::Time& _stamp, const tobas_msgs::Odometry& _setpoint, bool _landed) override;
  void update(const tobas_msgs::RCInput& _rcin, const tobas_msgs::Odometry& _odom, bool _landed) override;

private:
  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::msg::AileElevRudThrottle> cmd_pub_;
};
}  // namespace tobas::rc

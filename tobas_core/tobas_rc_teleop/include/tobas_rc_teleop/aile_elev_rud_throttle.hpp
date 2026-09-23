// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_command_msgs/msg/aile_elev_rud_throttle.hpp>

#include "./base_controller.hpp"

namespace tobas
{
namespace rc
{
class AileElevRudThrottleController : public BaseController
{
  using self = AileElevRudThrottleController;
  using super = BaseController;

  static constexpr double kMaxDeflectionCommand = 1.0;
  static constexpr double kMinDeflectionCommand = -1.0;

public:
  explicit AileElevRudThrottleController();

  bool requireHorizontalPosition() override;
  bool requireVerticalPosition() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(BaseNode* node, FlightMode mode) override;
  void reset(const builtin_interfaces::msg::Time& stamp, const tobas_msgs::Odometry& setpoint, bool landed) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed) override;

private:
  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::msg::AileElevRudThrottle> cmd_pub_;
};
}  // namespace rc
}  // namespace tobas

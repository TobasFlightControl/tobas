// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_command_msgs/msg/elev_aile_rud_throttle.hpp>

#include "./base_controller.hpp"

namespace tobas
{
namespace rc
{
class ElevAileRudThrottleController : public BaseController
{
  using self = ElevAileRudThrottleController;
  using super = BaseController;

  static constexpr double kDefaultDeflection = 10.0 / 180.0 * M_PI; // [rad]

public:
  explicit ElevAileRudThrottleController();

  bool requireHorizontalPosition() override;
  bool requireVerticalPosition() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(BaseNode* node, FlightMode mode) override;
  void reset(const builtin_interfaces::msg::Time& stamp, const tobas_msgs::Odometry& setpoint, bool landed) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed) override;

private:
  // ROS parameters.
  double min_elev_ = -kDefaultDeflection; // [rad]
  double max_elev_ = kDefaultDeflection;  // [rad]
  double min_aile_ = -kDefaultDeflection; // [rad]
  double max_aile_ = kDefaultDeflection;  // [rad]
  double min_rud_  = -kDefaultDeflection; // [rad]
  double max_rud_  = kDefaultDeflection;  // [rad]

  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::msg::ElevAileRudThrottle> cmd_pub_;

  bool minElevCb(const double& p);
  bool maxElevCb(const double& p);
  bool minAileCb(const double& p);
  bool maxAileCb(const double& p);
  bool minRudCb(const double& p);
  bool maxRudCb(const double& p);
};
}  // namespace rc
}  // namespace tobas

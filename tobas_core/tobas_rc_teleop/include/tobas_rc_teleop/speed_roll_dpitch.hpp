// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_command_msgs/msg/speed_roll_delta_pitch.hpp>

#include "./base_controller.hpp"

namespace tobas
{
namespace rc
{
class SpeedRollDeltaPitchController : public BaseController
{
  using self = SpeedRollDeltaPitchController;
  using super = BaseController;

public:
  explicit SpeedRollDeltaPitchController();

  bool requirePosition() override;
  bool requireVelocity() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(BaseNode* node, FlightMode mode) override;
  void reset(const builtin_interfaces::msg::Time& stamp, const tobas_msgs::Odometry& setpoint, bool landed) override;
  void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed) override;

private:
  // rosparams
  double min_speed_ = 0.0;                                 // [m/s]
  double max_speed_ = std::numeric_limits<double>::max();  // [m/s]
  double max_roll_;                                        // [rad]
  double max_dpitch_;                                      // [rad]
  double speed_expo_;
  double roll_expo_;
  double pitch_expo_;

  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::msg::SpeedRollDeltaPitch> cmd_pub_;

  bool minSpeedCb(const double& p);
  bool maxSpeedCb(const double& p);
  bool maxRollCb(const double& p);
  bool maxDeltaPitchCb(const double& p);
  bool speedExpoCb(const double& p);
  bool rollExpoCb(const double& p);
  bool pitchExpoCb(const double& p);
};
}  // namespace rc
}  // namespace tobas

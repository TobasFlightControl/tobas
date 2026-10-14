// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_command_msgs/msg/speed_roll_delta_pitch.hpp>

#include "./base_controller.hpp"

namespace tobas::rc
{
class SpeedRollDeltaPitchController : public BaseController
{
  using self = SpeedRollDeltaPitchController;

public:
  explicit SpeedRollDeltaPitchController();

  bool requireHorizontalPosition() override;
  bool requireVerticalPosition() override;
  bool requireAttitude() override;
  bool requireHeading() override;

  void initialize(BaseNode* _node, FlightMode _mode) override;
  void reset(const builtin_interfaces::msg::Time& _stamp, const tobas_msgs::Odometry& _setpoint, bool _landed) override;
  void update(const tobas_msgs::RCInput& _rcin, const tobas_msgs::Odometry& _odom, bool _landed) override;

private:
  // ROS parameters
  double min_speed_ = 0.0;                                 ///< [m/s]
  double max_speed_ = std::numeric_limits<double>::max();  ///< [m/s]
  double max_roll_;                                        ///< [rad]
  double max_dpitch_;                                      ///< [rad]
  double speed_expo_;
  double roll_expo_;
  double pitch_expo_;

  // PubSub
  ros2::PublisherPtr<tobas_command_msgs::msg::SpeedRollDeltaPitch> cmd_pub_;

  void minSpeedCb(double _p);
  void maxSpeedCb(double _p);
  void maxRollCb(double _p);
  void maxDeltaPitchCb(double _p);
  void speedExpoCb(double _p);
  void rollExpoCb(double _p);
  void pitchExpoCb(double _p);
};
}  // namespace tobas::rc

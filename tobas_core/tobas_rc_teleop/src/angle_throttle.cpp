// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rc_teleop/angle_throttle.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_constants/throttle.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas::rc
{
AngleThrottleController::AngleThrottleController()
{
}

bool AngleThrottleController::requireHorizontalPosition()
{
  return false;
}

bool AngleThrottleController::requireVerticalPosition()
{
  return false;
}

bool AngleThrottleController::requireAttitude()
{
  return true;
}

bool AngleThrottleController::requireHeading()
{
  return true;
}

void AngleThrottleController::initialize(BaseNode* _node, FlightMode _mode)
{
  _node->addDynamicDoubleParam(addMode("max_attitude", _mode), &self::maxAttitudeCb, this, 5.0, 9, 1, 16, " deg");
  _node->addDynamicDoubleParam(
    addMode("max_attitude_rate", _mode), &self::maxAttitudeRateCb, this, 15.0, 6, 1, 12, " dps");
  _node->addDynamicDoubleParam(
    addMode("max_heading_rate", _mode), &self::maxHeadingRateCb, this, 15.0, 6, 1, 12, " dps");
  _node->addDynamicDoubleParam(addMode("attitude_expo", _mode), &self::attitudeExpoCb, this, 5.0, -6, -20, 20);
  _node->addDynamicDoubleParam(addMode("heading_expo", _mode), &self::headingExpoCb, this, 5.0, -3, -20, 20);
  _node->addDynamicDoubleParam(addMode("throttle_expo", _mode), &self::throttleExpoCb, this, 5.0, 0, 0, 20);

  cmd_pub_ = _node->createPublisher<tobas_command_msgs::AngleThrottle>(topic::kAngleThrotCmd);
}

void AngleThrottleController::reset(
  const builtin_interfaces::msg::Time& _stamp,
  const tobas_msgs::Odometry& _setpoint,
  bool)
{
  t_last_rcin_ = _stamp;

  const auto [roll, pitch, yaw] = _setpoint.frame.M.getRPY();
  roll_filt_.resetCurrentTrajectoryPoint(roll);
  pitch_filt_.resetCurrentTrajectoryPoint(pitch);
  tar_yaw_ = yaw;
}

void AngleThrottleController::update(const tobas_msgs::RCInput& _rcin, const tobas_msgs::Odometry&, bool)
{
  // Update timestamp.
  const auto dt = (_rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = _rcin.header.stamp;

  // Create a command.
  auto cmd = std::make_unique<tobas_command_msgs::AngleThrottle>();
  cmd->header = _rcin.header;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;

  // Roll
  roll_filt_.setTargetPointAndUpdate(expoRemapDead(_rcin.roll, atti_expo_, -max_attitude_, max_attitude_), dt);
  cmd->angle.roll = roll_filt_.getTrajectoryPosition();

  // Pitch
  pitch_filt_.setTargetPointAndUpdate(expoRemapDead(_rcin.pitch, atti_expo_, -max_attitude_, max_attitude_), dt);
  cmd->angle.pitch = pitch_filt_.getTrajectoryPosition();

  // Yaw
  const auto yawrate = expoRemapDead(_rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_);
  tar_yaw_ += yawrate * dt;
  cmd->angle.yaw = tar_yaw_;

  // Throttle
  cmd->throttle = expo(remap(_rcin.throttle, kMinThrot, kMaxThrot), throt_expo_);

  // Publish the command.
  cmd_pub_->publish(std::move(cmd));
}

void AngleThrottleController::maxAttitudeCb(double _p)
{
  max_attitude_ = st::deg2rad(_p);
}

void AngleThrottleController::maxAttitudeRateCb(double _p)
{
  const auto max_atti_rate = st::deg2rad(_p);  // [rad/s]
  roll_filt_.setMaxVelocity(max_atti_rate);
  pitch_filt_.setMaxVelocity(max_atti_rate);
}

void AngleThrottleController::maxHeadingRateCb(double _p)
{
  max_head_rate_ = st::deg2rad(_p);
}

void AngleThrottleController::attitudeExpoCb(double _p)
{
  atti_expo_ = _p / kExpoScale;
}

void AngleThrottleController::headingExpoCb(double _p)
{
  head_expo_ = _p / kExpoScale;
}

void AngleThrottleController::throttleExpoCb(double _p)
{
  throt_expo_ = _p / kExpoScale;
}
}  // namespace tobas::rc

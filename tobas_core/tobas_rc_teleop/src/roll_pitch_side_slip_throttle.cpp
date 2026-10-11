// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rc_teleop/roll_pitch_side_slip_throttle.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_constants/throttle.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace rc
{
RollPitchSideSlipThrottleController::RollPitchSideSlipThrottleController()
{
}

bool RollPitchSideSlipThrottleController::requireHorizontalPosition()
{
  return false;
}

bool RollPitchSideSlipThrottleController::requireVerticalPosition()
{
  return false;
}

bool RollPitchSideSlipThrottleController::requireAttitude()
{
  return true;
}

bool RollPitchSideSlipThrottleController::requireHeading()
{
  return false;
}

void RollPitchSideSlipThrottleController::initialize(BaseNode* node, FlightMode mode)
{
  node->addDynamicDoubleParam(addMode("max_roll", mode), &self::maxRollCb, this, 5.0, 9, 1, 16, " deg");
  node->addDynamicDoubleParam(addMode("max_roll_rate", mode), &self::maxRollRateCb, this, 15.0, 6, 1, 12, " dps");
  node->addDynamicDoubleParam(addMode("max_pitch", mode), &self::maxPitchCb, this, 2.0, 7, 1, 14, " deg");
  node->addDynamicDoubleParam(addMode("max_pitch_rate", mode), &self::maxPitchRateCb, this, 5.0, 6, 1, 12, " dps");
  node->addDynamicDoubleParam(addMode("max_side_slip", mode), &self::maxSideSlipCb, this, 1.0, 6, 1, 12, " deg");
  node->addDynamicDoubleParam(addMode("max_side_slip_rate", mode), &self::maxSideSlipRateCb, this, 1.0, 6, 1, 12, " dps");
  node->addDynamicDoubleParam(addMode("attitude_expo", mode), &self::attitudeExpoCb, this, 5.0, -6, -20, 20);
  node->addDynamicDoubleParam(addMode("heading_expo", mode), &self::headingExpoCb, this, 5.0, -3, -20, 20);
  node->addDynamicDoubleParam(addMode("throttle_expo", mode), &self::throttleExpoCb, this, 5.0, 0, 0, 20);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::msg::RollPitchSideSlipThrottle>(
    topic::kRollPitchSideSlipThrottleCmd);
}

void RollPitchSideSlipThrottleController::reset(
  const builtin_interfaces::msg::Time& stamp,
  const tobas_msgs::Odometry& setpoint,
  bool)
{
  t_last_rcin_ = stamp;

  const auto [roll, pitch, yaw] = setpoint.frame.M.getRPY();
  roll_filt_.resetCurrentTrajectoryPoint(roll);
  pitch_filt_.resetCurrentTrajectoryPoint(pitch);
  side_slip_filt_.resetCurrentTrajectoryPoint(0.0);
}

void RollPitchSideSlipThrottleController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry&, bool)
{
  // Update timestamp.
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // Create a command.
  auto cmd = std::make_unique<tobas_command_msgs::msg::RollPitchSideSlipThrottle>();
  cmd->header = rcin.header;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;

  // Roll
  roll_filt_.setTargetPointAndUpdate(expoRemapDead(rcin.roll, atti_expo_, -max_roll_, max_roll_), dt);
  cmd->roll = roll_filt_.getTrajectoryPosition();

  // Pitch
  pitch_filt_.setTargetPointAndUpdate(expoRemapDead(rcin.pitch, atti_expo_, -max_pitch_, max_pitch_), dt);
  cmd->pitch = pitch_filt_.getTrajectoryPosition();

  // Beta
  side_slip_filt_.setTargetPointAndUpdate(expoRemapDead(rcin.yaw, head_expo_, -max_side_slip_, max_side_slip_), dt);
  cmd->side_slip = side_slip_filt_.getTrajectoryPosition();

  // Throttle
  cmd->throttle = expo(remap(rcin.throttle, kMinThrot, kMaxThrot), throt_expo_);

  // Publish the command.
  cmd_pub_->publish(std::move(cmd));
}

bool RollPitchSideSlipThrottleController::maxRollCb(const double& p)
{
  max_roll_ = st::deg2rad(p);
  return true;
}

bool RollPitchSideSlipThrottleController::maxRollRateCb(const double& p)
{
  const auto max_roll_rate = st::deg2rad(p);  // [rad/s]
  roll_filt_.setMaxVelocity(max_roll_rate);
  return true;
}

bool RollPitchSideSlipThrottleController::maxPitchCb(const double& p)
{
  max_pitch_ = st::deg2rad(p);
  return true;
}

bool RollPitchSideSlipThrottleController::maxPitchRateCb(const double& p)
{
  const auto max_pitch_rate = st::deg2rad(p);  // [rad/s]
  pitch_filt_.setMaxVelocity(max_pitch_rate);
  return true;
}

bool RollPitchSideSlipThrottleController::maxSideSlipCb(const double& p)
{
  max_side_slip_ = st::deg2rad(p);
  return true;
}

bool RollPitchSideSlipThrottleController::maxSideSlipRateCb(const double& p)
{
  const auto max_side_slip_rate = st::deg2rad(p); // [rad/s]
  side_slip_filt_.setMaxVelocity(max_side_slip_rate);
  return true;
}

bool RollPitchSideSlipThrottleController::attitudeExpoCb(const double& p)
{
  atti_expo_ = p / kExpoScale;
  return true;
}

bool RollPitchSideSlipThrottleController::headingExpoCb(const double& p)
{
  head_expo_ = p / kExpoScale;
  return true;
}

bool RollPitchSideSlipThrottleController::throttleExpoCb(const double& p)
{
  throt_expo_ = p / kExpoScale;
  return true;
}
}  // namespace rc
}  // namespace tobas

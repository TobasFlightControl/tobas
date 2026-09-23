// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rc_teleop/elev_aile_rud_throttle.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_constants/throttle.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace rc
{
ElevAileRudThrottleController::ElevAileRudThrottleController()
{
}

bool ElevAileRudThrottleController::requireHorizontalPosition()
{
  return false;
}

bool ElevAileRudThrottleController::requireVerticalPosition()
{
  return false;
}

bool ElevAileRudThrottleController::requireAttitude()
{
  return false;
}

bool ElevAileRudThrottleController::requireHeading()
{
  return false;
}

void ElevAileRudThrottleController::initialize(BaseNode* node, FlightMode)
{
  cmd_pub_ = node->createPublisher<tobas_command_msgs::msg::ElevAileRudThrottle>(topic::kElevAileRudThrottleCmd);
}

void ElevAileRudThrottleController::reset(const builtin_interfaces::msg::Time&, const tobas_msgs::Odometry&, bool)
{
}

void ElevAileRudThrottleController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry&, bool)
{
  // Create command.
  auto cmd = std::make_unique<tobas_command_msgs::msg::ElevAileRudThrottle>();
  cmd->header = rcin.header;

  // TODO exp, remapDead
  cmd->elevator = remap(rcin.pitch, cmd->MIN_DEFLECTION, cmd->MAX_DEFLECTION);
  cmd->aileron = remap(rcin.roll, cmd->MIN_DEFLECTION, cmd->MAX_DEFLECTION);
  cmd->rudder = remap(rcin.yaw, cmd->MIN_DEFLECTION, cmd->MAX_DEFLECTION);
  cmd->throttle = remap(rcin.throttle, kMinThrot, kMaxThrot);

  // Publish command.
  cmd_pub_->publish(std::move(cmd));
}
}  // namespace rc
}  // namespace tobas

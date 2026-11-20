// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rc_teleop/aile_elev_rud_throttle.hpp"

#include <tobas_constants/ros.hpp>
#include <tobas_constants/throttle.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace rc
{
AileElevRudThrottleController::AileElevRudThrottleController()
{
}

bool AileElevRudThrottleController::requireHorizontalPosition() const
{
  return false;
}

bool AileElevRudThrottleController::requireVerticalPosition() const
{
  return false;
}

bool AileElevRudThrottleController::requireAttitude() const
{
  return false;
}

bool AileElevRudThrottleController::requireHeading() const
{
  return false;
}

void AileElevRudThrottleController::initialize(BaseNode* _node, FlightMode)
{
  cmd_pub_ = _node->createPublisher<tobas_command_msgs::msg::AileElevRudThrottle>(topic::kAileElevRudThrottleCmd);
}

void AileElevRudThrottleController::reset(const builtin_interfaces::msg::Time&, const tobas_msgs::Odometry&, bool)
{
}

void AileElevRudThrottleController::update(const tobas_msgs::RCInput& _rcin, const tobas_msgs::Odometry&, bool)
{
  // Create command.
  auto cmd = std::make_unique<tobas_command_msgs::msg::AileElevRudThrottle>();
  cmd->header = _rcin.header;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;

  // TODO exp, remapDead
  cmd->elevator = remap(_rcin.pitch, cmd->MIN_DEFLECTION, cmd->MAX_DEFLECTION);
  cmd->aileron = remap(_rcin.roll, cmd->MIN_DEFLECTION, cmd->MAX_DEFLECTION);
  cmd->rudder = remap(_rcin.yaw, cmd->MIN_DEFLECTION, cmd->MAX_DEFLECTION);
  cmd->throttle = remap(_rcin.throttle, kMinThrot, kMaxThrot);

  // Publish command.
  cmd_pub_->publish(std::move(cmd));
}
}  // namespace rc
}  // namespace tobas

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

void ElevAileRudThrottleController::initialize(BaseNode* node, FlightMode mode)
{
  node->addDynamicDoubleParam(addMode("min_elev", mode), &self::minElevCb, this, 1.0, -15, -30, -1, " deg");
  node->addDynamicDoubleParam(addMode("max_elev", mode), &self::maxElevCb, this, 1.0,  15,   1, 30, " deg");
  node->addDynamicDoubleParam(addMode("min_aile", mode), &self::minAileCb, this, 1.0, -15, -30, -1, " deg");
  node->addDynamicDoubleParam(addMode("max_aile", mode), &self::maxAileCb, this, 1.0,  15,   1, 30, " deg");
  node->addDynamicDoubleParam(addMode("min_rud", mode), &self::minRudCb, this, 1.0, -15, -30, -1, " deg");
  node->addDynamicDoubleParam(addMode("max_rud", mode), &self::maxRudCb, this, 1.0,  15,   1, 30, " deg");

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
  cmd->elevator = remap(rcin.pitch, min_elev_, max_elev_);
  cmd->aileron = remap(rcin.roll, min_aile_, max_aile_);
  cmd->rudder = remap(rcin.yaw, min_rud_, max_rud_);
  cmd->throttle = remap(rcin.throttle, kMinThrot, kMaxThrot);

  // Publish command.
  cmd_pub_->publish(std::move(cmd));
}

bool ElevAileRudThrottleController::minElevCb(const double& p)
{
  min_elev_ = st::deg2rad(p);
  return true;
}

bool ElevAileRudThrottleController::maxElevCb(const double& p)
{
  max_elev_ = st::deg2rad(p);
  return true;
}

bool ElevAileRudThrottleController::minAileCb(const double& p)
{
  min_aile_ = st::deg2rad(p);
  return true;
}

bool ElevAileRudThrottleController::maxAileCb(const double& p)
{
  max_aile_ = st::deg2rad(p);
  return true;
}

bool ElevAileRudThrottleController::minRudCb(const double& p)
{
  min_rud_ = st::deg2rad(p);
  return true;
}

bool ElevAileRudThrottleController::maxRudCb(const double& p)
{
  max_rud_ = st::deg2rad(p);
  return true;
}
}  // namespace rc
}  // namespace tobas

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rc_teleop/rate_throttle.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_constants/throttle.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace rc
{
RateThrottleController::RateThrottleController()
{
}

bool RateThrottleController::requireHorizontalPosition()
{
  return false;
}

bool RateThrottleController::requireVerticalPosition()
{
  return false;
}

bool RateThrottleController::requireAttitude()
{
  return false;
}

bool RateThrottleController::requireHeading()
{
  return false;
}

void RateThrottleController::initialize(BaseNode* node, FlightMode mode)
{
  node->addDynamicDoubleParam(
    addMode("max_attitude_rate", mode), &self::maxAttitudeRateCb, this, 45.0, 8, 1, 16, " dps");
  node->addDynamicDoubleParam(addMode("max_heading_rate", mode), &self::maxHeadingRateCb, this, 45.0, 8, 1, 16, " dps");
  node->addDynamicDoubleParam(addMode("attitude_expo", mode), &self::attitudeExpoCb, this, 5.0, -6, -20, 20);
  node->addDynamicDoubleParam(addMode("heading_expo", mode), &self::headingExpoCb, this, 5.0, -3, -20, 20);
  node->addDynamicDoubleParam(addMode("throttle_expo", mode), &self::throttleExpoCb, this, 5.0, 0, 0, 20);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::RateThrottle>(topic::kRateThrotCmd);
}

void RateThrottleController::reset(const builtin_interfaces::msg::Time&, const tobas_msgs::Odometry&, bool)
{
}

void RateThrottleController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry&, bool)
{
  // Create command.
  auto cmd = std::make_unique<tobas_command_msgs::RateThrottle>();
  cmd->header = rcin.header;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
  cmd->rate.x(expoRemap(rcin.roll, atti_expo_, -max_atti_rate_, max_atti_rate_));
  cmd->rate.y(expoRemap(rcin.pitch, atti_expo_, -max_atti_rate_, max_atti_rate_));
  cmd->rate.z(expoRemap(rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_));
  cmd->throttle = expo(remap(rcin.throttle, kMinThrot, kMaxThrot), throt_expo_);

  // Publish command.
  cmd_pub_->publish(std::move(cmd));
}

void RateThrottleController::maxAttitudeRateCb(double p)
{
  max_atti_rate_ = st::deg2rad(p);
}

void RateThrottleController::maxHeadingRateCb(double p)
{
  max_head_rate_ = st::deg2rad(p);
}

void RateThrottleController::attitudeExpoCb(double p)
{
  atti_expo_ = p / kExpoScale;
}

void RateThrottleController::headingExpoCb(double p)
{
  head_expo_ = p / kExpoScale;
}

void RateThrottleController::throttleExpoCb(double p)
{
  throt_expo_ = p / kExpoScale;
}
}  // namespace rc
}  // namespace tobas

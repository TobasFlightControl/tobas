// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rc_teleop/rate_throttle_vector.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_constants/throttle.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace rc
{
RateThrottleVectorController::RateThrottleVectorController()
{
}

bool RateThrottleVectorController::requireHorizontalPosition()
{
  return false;
}

bool RateThrottleVectorController::requireVerticalPosition()
{
  return false;
}

bool RateThrottleVectorController::requireAttitude()
{
  return false;
}

bool RateThrottleVectorController::requireHeading()
{
  return false;
}

void RateThrottleVectorController::initialize(BaseNode* _node, FlightMode _mode)
{
  _node->addDynamicDoubleParam(
    addMode("max_attitude_rate", _mode), &self::maxAttitudeRateCb, this, 45.0, 8, 1, 16, " dps");
  _node->addDynamicDoubleParam(
    addMode("max_heading_rate", _mode), &self::maxHeadingRateCb, this, 45.0, 8, 1, 16, " dps");
  _node->addDynamicDoubleParam(
    addMode("max_thrust_angle", _mode), &self::maxThrustAngleCb, this, 15.0, 6, 1, 12, " deg");
  _node->addDynamicDoubleParam(addMode("attitude_expo", _mode), &self::attitudeExpoCb, this, 5.0, -6, -20, 20);
  _node->addDynamicDoubleParam(addMode("heading_expo", _mode), &self::headingExpoCb, this, 5.0, -3, -20, 20);
  _node->addDynamicDoubleParam(addMode("throttle_expo", _mode), &self::throttleExpoCb, this, 5.0, 0, 0, 20);

  cmd_pub_ = _node->createPublisher<tobas_command_msgs::RateThrottleVector>(topic::kRateThrotVectorCmd);
}

void RateThrottleVectorController::reset(const builtin_interfaces::msg::Time&, const tobas_msgs::Odometry&, bool)
{
}

void RateThrottleVectorController::update(const tobas_msgs::RCInput& _rcin, const tobas_msgs::Odometry&, bool)
{
  auto cmd = std::make_unique<tobas_command_msgs::RateThrottleVector>();
  cmd->header = _rcin.header;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;

  if (_rcin.sub_mode) {  // Translation mode
    cmd->rate.y(0.0);
    cmd->thrust_angle = remapDead(_rcin.pitch, -max_thrust_angle_, max_thrust_angle_);
  }
  else {  // Rotation mode
    cmd->rate.y(expoRemap(_rcin.pitch, atti_expo_, -max_atti_rate_, max_atti_rate_));
    cmd->thrust_angle = 0.0;
  }

  cmd->rate.x(expoRemap(_rcin.roll, atti_expo_, -max_atti_rate_, max_atti_rate_));
  cmd->rate.z(expoRemap(_rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_));
  cmd->throttle = expo(remap(_rcin.throttle, kMinThrot, kMaxThrot), throt_expo_);

  cmd_pub_->publish(std::move(cmd));
}

void RateThrottleVectorController::maxAttitudeRateCb(double _p)
{
  max_atti_rate_ = st::deg2rad(_p);
}

void RateThrottleVectorController::maxHeadingRateCb(double _p)
{
  max_head_rate_ = st::deg2rad(_p);
}

void RateThrottleVectorController::maxThrustAngleCb(double _p)
{
  max_thrust_angle_ = st::deg2rad(_p);
}

void RateThrottleVectorController::attitudeExpoCb(double _p)
{
  atti_expo_ = _p / kExpoScale;
}

void RateThrottleVectorController::headingExpoCb(double _p)
{
  head_expo_ = _p / kExpoScale;
}

void RateThrottleVectorController::throttleExpoCb(double _p)
{
  throt_expo_ = _p / kExpoScale;
}
}  // namespace rc
}  // namespace tobas

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rc_teleop/speed_roll_dpitch.hpp"

#include <cassert>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_constants/throttle.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas::rc
{
SpeedRollDeltaPitchController::SpeedRollDeltaPitchController()
{
}

bool SpeedRollDeltaPitchController::requireHorizontalPosition()
{
  return true;
}

bool SpeedRollDeltaPitchController::requireVerticalPosition()
{
  return true;
}

bool SpeedRollDeltaPitchController::requireAttitude()
{
  return true;
}

bool SpeedRollDeltaPitchController::requireHeading()
{
  return false;
}

void SpeedRollDeltaPitchController::initialize(BaseNode* _node, FlightMode _mode)
{
  _node->addDynamicDoubleParam(addMode("min_speed", _mode), &self::minSpeedCb, this, 0.5, 10, 1, 20, " m/s");
  _node->addDynamicDoubleParam(addMode("max_speed", _mode), &self::maxSpeedCb, this, 0.5, 40, 1, 80, " m/s");
  _node->addDynamicDoubleParam(addMode("max_roll", _mode), &self::maxRollCb, this, 15.0, 6, 1, 12, " deg");
  _node->addDynamicDoubleParam(addMode("max_delta_pitch", _mode), &self::maxDeltaPitchCb, this, 5.0, 9, 1, 18, " deg");
  _node->addDynamicDoubleParam(addMode("speed_expo", _mode), &self::speedExpoCb, this, 5.0, 0, 0, 20);
  _node->addDynamicDoubleParam(addMode("roll_expo", _mode), &self::rollExpoCb, this, 5.0, 0, -20, 20);
  _node->addDynamicDoubleParam(addMode("pitch_expo", _mode), &self::pitchExpoCb, this, 5.0, 0, -20, 20);

  cmd_pub_ = _node->createPublisher<tobas_command_msgs::msg::SpeedRollDeltaPitch>(topic::kSpeedRollDpitchCmd);
}

void SpeedRollDeltaPitchController::reset(const builtin_interfaces::msg::Time&, const tobas_msgs::Odometry&, bool)
{
}

void SpeedRollDeltaPitchController::update(const tobas_msgs::RCInput& _rcin, const tobas_msgs::Odometry&, bool)
{
  // Create command.
  auto cmd = std::make_unique<tobas_command_msgs::msg::SpeedRollDeltaPitch>();
  cmd->header = _rcin.header;

  // TODO: Consider the airframe speed limit.
  const auto throttle = expo(remap(_rcin.throttle, kMinThrot, kMaxThrot), speed_expo_);
  cmd->speed = math::remap(throttle, kMinThrot, kMaxThrot, min_speed_, max_speed_);

  cmd->roll = remapDead(_rcin.roll, -max_roll_, max_roll_);
  cmd->delta_pitch = remapDead(_rcin.pitch, -max_dpitch_, max_dpitch_);

  // Publish command.
  cmd_pub_->publish(std::move(cmd));
}

void SpeedRollDeltaPitchController::minSpeedCb(double _p)
{
  assert(_p < max_speed_);

  min_speed_ = _p;
}

void SpeedRollDeltaPitchController::maxSpeedCb(double _p)
{
  assert(_p > min_speed_);

  max_speed_ = _p;
}

void SpeedRollDeltaPitchController::maxRollCb(double _p)
{
  max_roll_ = st::deg2rad(_p);
}

void SpeedRollDeltaPitchController::maxDeltaPitchCb(double _p)
{
  max_dpitch_ = st::deg2rad(_p);
}

void SpeedRollDeltaPitchController::speedExpoCb(double _p)
{
  speed_expo_ = _p / kExpoScale;
}

void SpeedRollDeltaPitchController::rollExpoCb(double _p)
{
  roll_expo_ = _p / kExpoScale;
}

void SpeedRollDeltaPitchController::pitchExpoCb(double _p)
{
  pitch_expo_ = _p / kExpoScale;
}
}  // namespace tobas::rc

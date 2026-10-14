// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rc_teleop/angle_throttle_vector.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_constants/throttle.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas::rc
{
AngleThrottleVectorController::AngleThrottleVectorController()
{
}

bool AngleThrottleVectorController::requireHorizontalPosition()
{
  return false;
}

bool AngleThrottleVectorController::requireVerticalPosition()
{
  return false;
}

bool AngleThrottleVectorController::requireAttitude()
{
  return true;
}

bool AngleThrottleVectorController::requireHeading()
{
  return true;
}

void AngleThrottleVectorController::initialize(BaseNode* _node, FlightMode _mode)
{
  _node->addDynamicDoubleParam(addMode("max_roll", _mode), &self::maxRollCb, this, 5.0, 9, 1, 16, " deg");
  _node->addDynamicDoubleParam(addMode("max_roll_rate", _mode), &self::maxRollRateCb, this, 15.0, 6, 1, 12, " dps");
  _node->addDynamicDoubleParam(addMode("max_pitch", _mode), &self::maxPitchCb, this, 15.0, 6, 1, 12, " deg");
  _node->addDynamicDoubleParam(addMode("max_pitch_rate", _mode), &self::maxPitchRateCb, this, 15.0, 6, 1, 12, " dps");
  _node->addDynamicDoubleParam(addMode("max_yaw_rate", _mode), &self::maxYawRateCb, this, 15.0, 6, 1, 12, " dps");
  _node->addDynamicDoubleParam(
    addMode("max_thrust_angle", _mode), &self::maxThrustAngleCb, this, 15.0, 6, 1, 12, " deg");
  _node->addDynamicDoubleParam(
    addMode("max_thrust_angle_rate", _mode), &self::maxThrustAngleRateCb, this, 15.0, 6, 1, 12, " dps");
  _node->addDynamicDoubleParam(addMode("roll_expo", _mode), &self::rollExpoCb, this, 5.0, -6, -20, 20);
  _node->addDynamicDoubleParam(addMode("yaw_expo", _mode), &self::yawExpoCb, this, 5.0, -3, -20, 20);
  _node->addDynamicDoubleParam(addMode("throttle_expo", _mode), &self::throttleExpoCb, this, 5.0, 0, 0, 20);
  _node->addDynamicDoubleParam(addMode("thrust_angle_expo", _mode), &self::thrustAngleExpoCb, this, 5.0, 0, -20, 20);

  cmd_pub_ = _node->createPublisher<tobas_command_msgs::AngleThrottleVector>(topic::kAngleThrotVectorCmd);
}

void AngleThrottleVectorController::reset(
  const builtin_interfaces::msg::Time& _stamp,
  const tobas_msgs::Odometry& _setpoint,
  bool)
{
  t_last_rcin_ = _stamp;

  const auto [roll, pitch, yaw] = _setpoint.frame.M.getRPY();
  roll_filt_.resetCurrentTrajectoryPoint(roll);
  pitch_filt_.resetCurrentTrajectoryPoint(pitch);
  thrust_angle_filt_.resetCurrentTrajectoryPoint(-pitch);
  tar_yaw_ = yaw;
}

void AngleThrottleVectorController::update(const tobas_msgs::RCInput& _rcin, const tobas_msgs::Odometry&, bool)
{
  // Update timestamp.
  const auto dt = (_rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = _rcin.header.stamp;

  // Create a command.
  auto cmd = std::make_unique<tobas_command_msgs::AngleThrottleVector>();
  cmd->header = _rcin.header;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;

  // Roll
  roll_filt_.setTargetPointAndUpdate(expoRemapDead(_rcin.roll, roll_expo_, -max_roll_, max_roll_), dt);
  cmd->angle.roll = roll_filt_.getTrajectoryPosition();

  // Yaw
  const auto yawrate = expoRemapDead(_rcin.yaw, yaw_expo_, -max_yaw_rate_, max_yaw_rate_);
  tar_yaw_ += yawrate * dt;
  cmd->angle.yaw = tar_yaw_;

  // Throttle
  cmd->throttle = expo(remap(_rcin.throttle, kMinThrot, kMaxThrot), throt_expo_);

  // Pitch & Thrust Angle
  if (_rcin.sub_mode)  // Translation mode
  {
    pitch_filt_.setTargetPointAndUpdate(0.0, dt);
    thrust_angle_filt_.setTargetPointAndUpdate(
      expoRemapDead(_rcin.pitch, thrust_angle_expo_, -max_thrust_angle_, max_thrust_angle_), dt);
  }
  else  // Rotation mode
  {
    const auto tar_pitch = remapDead(_rcin.pitch, -max_pitch_, max_pitch_);
    pitch_filt_.setTargetPointAndUpdate(tar_pitch, dt);
    thrust_angle_filt_.setTargetPointAndUpdate(-tar_pitch, dt);
  }
  cmd->angle.pitch = pitch_filt_.getTrajectoryPosition();
  cmd->thrust_angle = thrust_angle_filt_.getTrajectoryPosition();

  // Publish the command.
  cmd_pub_->publish(std::move(cmd));
}

void AngleThrottleVectorController::maxRollCb(double _p)
{
  max_roll_ = st::deg2rad(_p);
}

void AngleThrottleVectorController::maxRollRateCb(double _p)
{
  roll_filt_.setMaxVelocity(st::deg2rad(_p));
}

void AngleThrottleVectorController::maxPitchCb(double _p)
{
  max_pitch_ = st::deg2rad(_p);
}

void AngleThrottleVectorController::maxPitchRateCb(double _p)
{
  pitch_filt_.setMaxVelocity(st::deg2rad(_p));
}

void AngleThrottleVectorController::maxYawRateCb(double _p)
{
  max_yaw_rate_ = st::deg2rad(_p);
}

void AngleThrottleVectorController::maxThrustAngleCb(double _p)
{
  max_thrust_angle_ = st::deg2rad(_p);
}

void AngleThrottleVectorController::maxThrustAngleRateCb(double _p)
{
  thrust_angle_filt_.setMaxVelocity(st::deg2rad(_p));
}

void AngleThrottleVectorController::rollExpoCb(double _p)
{
  roll_expo_ = _p / kExpoScale;
}

void AngleThrottleVectorController::yawExpoCb(double _p)
{
  yaw_expo_ = _p / kExpoScale;
}

void AngleThrottleVectorController::throttleExpoCb(double _p)
{
  throt_expo_ = _p / kExpoScale;
}

void AngleThrottleVectorController::thrustAngleExpoCb(double _p)
{
  thrust_angle_expo_ = _p / kExpoScale;
}
}  // namespace tobas::rc

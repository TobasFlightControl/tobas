// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rc_teleop/accel_pitch_yaw.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace rc
{
AccelPitchYawController::AccelPitchYawController()
{
}

bool AccelPitchYawController::requireHorizontalPosition()
{
  return false;
}

bool AccelPitchYawController::requireVerticalPosition()
{
  return false;
}

bool AccelPitchYawController::requireAttitude()
{
  return true;
}

bool AccelPitchYawController::requireHeading()
{
  return true;
}

void AccelPitchYawController::initialize(BaseNode* _node, FlightMode _mode)
{
  _node->addDynamicDoubleParam(
    addMode("max_horizontal_accel", _mode), &self::maxHorizontalAccelCb, this, 0.5, 10, 1, 20, " m/s^2");
  _node->addDynamicDoubleParam(
    addMode("max_horizontal_jerk", _mode), &self::maxHorizontalJerkCb, this, 5.0, 8, 1, 20, " m/s^3");
  _node->addDynamicDoubleParam(
    addMode("max_vertical_accel", _mode), &self::maxVerticalAccelCb, this, 0.5, 16, 1, 20, " m/s^2");
  _node->addDynamicDoubleParam(addMode("max_pitch", _mode), &self::maxPitchCb, this, 15.0, 6, 1, 12, " deg");
  _node->addDynamicDoubleParam(addMode("max_pitch_rate", _mode), &self::maxPitchRateCb, this, 15.0, 6, 1, 12, " dps");
  _node->addDynamicDoubleParam(addMode("max_yaw_rate", _mode), &self::maxYawRateCb, this, 15.0, 6, 1, 12, " dps");
  _node->addDynamicDoubleParam(
    addMode("horizontal_accel_expo", _mode), &self::horizontalAccelExpoCb, this, 5.0, -6, -20, 20);
  _node->addDynamicDoubleParam(addMode("vertical_accel_expo", _mode), &self::verticalAccelExpoCb, this, 5.0, 0, -20, 20);
  _node->addDynamicDoubleParam(addMode("pitch_expo", _mode), &self::pitchExpoCb, this, 5.0, 0, -20, 20);
  _node->addDynamicDoubleParam(addMode("yaw_expo", _mode), &self::yawExpoCb, this, 5.0, -3, -20, 20);

  cmd_pub_ = _node->createPublisher<tobas_command_msgs::AccelPitchYaw>(topic::kAccelPitchYawCmd);
}

void AccelPitchYawController::reset(
  const builtin_interfaces::msg::Time& _stamp,
  const tobas_msgs::Odometry& _setpoint,
  bool)
{
  t_last_rcin_ = _stamp;

  ax_filt_.resetCurrentTrajectoryPoint(0.0);
  ay_filt_.resetCurrentTrajectoryPoint(0.0);

  pitch_filt_.resetCurrentTrajectoryPoint(_setpoint.frame.M.getPitch());
  tar_yaw_ = _setpoint.frame.M.getYaw();
}

void AccelPitchYawController::update(const tobas_msgs::RCInput& _rcin, const tobas_msgs::Odometry&, bool)
{
  // Update timestamp.
  const auto dt = (_rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = _rcin.header.stamp;

  // Accel-X & Pitch
  if (_rcin.sub_mode)  // Translation mode
  {
    ax_filt_.setTargetPointAndUpdate(expoRemap(_rcin.pitch, hor_acc_expo_, -max_hor_acc_, max_hor_acc_), dt);
    pitch_filt_.setTargetPointAndUpdate(0.0, dt);
  }
  else  // Rotation mode
  {
    ax_filt_.setTargetPointAndUpdate(0.0, dt);
    pitch_filt_.setTargetPointAndUpdate(expoRemapDead(_rcin.pitch, pitch_expo_, -max_pitch_, max_pitch_), dt);
  }

  // Accel-Y
  ay_filt_.setTargetPointAndUpdate(-expoRemap(_rcin.roll, hor_acc_expo_, -max_hor_acc_, max_hor_acc_), dt);

  // Accel-Z
  const auto az = expoRemap(_rcin.throttle, ver_acc_expo_, -max_ver_acc_, max_ver_acc_);

  // Yaw
  const auto yawrate = expoRemapDead(_rcin.yaw, yaw_expo_, -max_yaw_rate_, max_yaw_rate_);
  tar_yaw_ += yawrate * dt;

  // Compute the acceleration wrt. the world frame.
  const kdl::Vector tar_acc_G(ax_filt_.getTrajectoryPosition(), ay_filt_.getTrajectoryPosition(), az);
  const auto tar_acc_W = kdl::Rotation::RotZ(tar_yaw_) * tar_acc_G;

  // Create a command.
  auto cmd = std::make_unique<tobas_command_msgs::AccelPitchYaw>();
  cmd->header = _rcin.header;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
  cmd->accel = tar_acc_W;
  cmd->pitch = pitch_filt_.getTrajectoryPosition();
  cmd->yaw = tar_yaw_;

  // Publish the command.
  cmd_pub_->publish(std::move(cmd));
}

void AccelPitchYawController::maxHorizontalAccelCb(double _p)
{
  max_hor_acc_ = _p;
}

void AccelPitchYawController::maxHorizontalJerkCb(double _p)
{
  ax_filt_.setMaxVelocity(_p);
  ay_filt_.setMaxVelocity(_p);
}

void AccelPitchYawController::maxVerticalAccelCb(double _p)
{
  max_ver_acc_ = _p;
}

void AccelPitchYawController::maxPitchCb(double _p)
{
  max_pitch_ = st::deg2rad(_p);
}

void AccelPitchYawController::maxPitchRateCb(double _p)
{
  pitch_filt_.setMaxVelocity(st::deg2rad(_p));
}

void AccelPitchYawController::maxYawRateCb(double _p)
{
  max_yaw_rate_ = st::deg2rad(_p);
}

void AccelPitchYawController::horizontalAccelExpoCb(double _p)
{
  hor_acc_expo_ = _p / kExpoScale;
}

void AccelPitchYawController::verticalAccelExpoCb(double _p)
{
  ver_acc_expo_ = _p / kExpoScale;
}

void AccelPitchYawController::pitchExpoCb(double _p)
{
  pitch_expo_ = _p / kExpoScale;
}

void AccelPitchYawController::yawExpoCb(double _p)
{
  yaw_expo_ = _p / kExpoScale;
}
}  // namespace rc
}  // namespace tobas

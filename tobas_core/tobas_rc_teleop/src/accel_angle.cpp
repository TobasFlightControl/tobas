// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rc_teleop/accel_angle.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace rc
{
AccelAngleController::AccelAngleController()
{
}

bool AccelAngleController::requireHorizontalPosition()
{
  return false;
}

bool AccelAngleController::requireVerticalPosition()
{
  return false;
}

bool AccelAngleController::requireAttitude()
{
  return true;
}

bool AccelAngleController::requireHeading()
{
  return true;
}

void AccelAngleController::initialize(BaseNode* _node, FlightMode _mode)
{
  _node->addDynamicDoubleParam(
    addMode("max_horizontal_accel", _mode), &self::maxHorizontalAccelCb, this, 0.5, 10, 1, 20, " m/s^2");
  _node->addDynamicDoubleParam(
    addMode("max_horizontal_jerk", _mode), &self::maxHorizontalJerkCb, this, 5.0, 8, 1, 20, " m/s^3");
  _node->addDynamicDoubleParam(
    addMode("max_vertical_accel", _mode), &self::maxVerticalAccelCb, this, 0.5, 16, 1, 20, " m/s^2");
  _node->addDynamicDoubleParam(addMode("max_attitude", _mode), &self::maxAttitudeCb, this, 15.0, 6, 1, 12, " deg");
  _node->addDynamicDoubleParam(
    addMode("max_attitude_rate", _mode), &self::maxAttitudeRateCb, this, 15.0, 6, 1, 12, " dps");
  _node->addDynamicDoubleParam(
    addMode("max_heading_rate", _mode), &self::maxHeadingRateCb, this, 15.0, 6, 1, 12, " dps");
  _node->addDynamicDoubleParam(
    addMode("horizontal_accel_expo", _mode), &self::horizontalAccelExpoCb, this, 5.0, -6, -20, 20);
  _node->addDynamicDoubleParam(addMode("vertical_accel_expo", _mode), &self::verticalAccelExpoCb, this, 5.0, 0, -20, 20);
  _node->addDynamicDoubleParam(addMode("attitude_expo", _mode), &self::attitudeExpoCb, this, 5.0, 0, -20, 20);
  _node->addDynamicDoubleParam(addMode("heading_expo", _mode), &self::headingExpoCb, this, 5.0, -3, -20, 20);

  accel_pub_ = _node->createPublisher<tobas_command_msgs::Accel>(topic::kAccelCmd);
  angle_pub_ = _node->createPublisher<tobas_command_msgs::Angle>(topic::kAngleCmd);
}

void AccelAngleController::reset(const builtin_interfaces::msg::Time& _stamp, const tobas_msgs::Odometry& _setpoint, bool)
{
  t_last_rcin_ = _stamp;

  ax_filt_.resetCurrentTrajectoryPoint(0.0);
  ay_filt_.resetCurrentTrajectoryPoint(0.0);

  const auto [roll, pitch, yaw] = _setpoint.frame.M.getRPY();
  roll_filt_.resetCurrentTrajectoryPoint(roll);
  pitch_filt_.resetCurrentTrajectoryPoint(pitch);
  tar_yaw_ = yaw;
}

void AccelAngleController::update(const tobas_msgs::RCInput& _rcin, const tobas_msgs::Odometry&, bool)
{
  // Update timestamp.
  const auto dt = (_rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = _rcin.header.stamp;

  // Horizontal acceleration & Attitude
  if (_rcin.sub_mode)  // Translation mode
  {
    ax_filt_.setTargetPointAndUpdate(expoRemap(_rcin.pitch, hor_acc_expo_, -max_hor_acc_, max_hor_acc_), dt);
    ay_filt_.setTargetPointAndUpdate(-expoRemap(_rcin.roll, hor_acc_expo_, -max_hor_acc_, max_hor_acc_), dt);
    roll_filt_.setTargetPointAndUpdate(0.0, dt);
    pitch_filt_.setTargetPointAndUpdate(0.0, dt);
  }
  else  // Rotation mode
  {
    roll_filt_.setTargetPointAndUpdate(expoRemapDead(_rcin.roll, atti_expo_, -max_attitude_, max_attitude_), dt);
    pitch_filt_.setTargetPointAndUpdate(expoRemapDead(_rcin.pitch, atti_expo_, -max_attitude_, max_attitude_), dt);
    ax_filt_.setTargetPointAndUpdate(0.0, dt);
    ay_filt_.setTargetPointAndUpdate(0.0, dt);
  }

  // Vertical acceleration
  const auto az = expoRemap(_rcin.throttle, ver_acc_expo_, -max_ver_acc_, max_ver_acc_);

  // Yaw
  const auto yawrate = expoRemapDead(_rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_);
  tar_yaw_ += yawrate * dt;

  // Compute the acceleration wrt. the world frame.
  const kdl::Vector tar_acc_G(ax_filt_.getTrajectoryPosition(), ay_filt_.getTrajectoryPosition(), az);
  const auto tar_acc_W = kdl::Rotation::RotZ(tar_yaw_) * tar_acc_G;

  // Publish commands.
  publishAccel(_rcin.header.stamp, tar_acc_W);
  publishAngle(_rcin.header.stamp, roll_filt_.getTrajectoryPosition(), pitch_filt_.getTrajectoryPosition(), tar_yaw_);
}

void AccelAngleController::publishAccel(const builtin_interfaces::msg::Time& _stamp, const kdl::Vector& _acc)
{
  auto cmd = std::make_unique<tobas_command_msgs::Accel>();
  cmd->header.stamp = _stamp;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
  cmd->accel = _acc;

  accel_pub_->publish(std::move(cmd));
}

void AccelAngleController::publishAngle(
  const builtin_interfaces::msg::Time& _stamp,
  double _roll,
  double _pitch,
  double _yaw)
{
  auto cmd = std::make_unique<tobas_command_msgs::Angle>();
  cmd->header.stamp = _stamp;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
  cmd->angle.set(_roll, _pitch, _yaw);

  angle_pub_->publish(std::move(cmd));
}

void AccelAngleController::maxHorizontalAccelCb(double _p)
{
  max_hor_acc_ = _p;
}

void AccelAngleController::maxHorizontalJerkCb(double _p)
{
  ax_filt_.setMaxVelocity(_p);
  ay_filt_.setMaxVelocity(_p);
}

void AccelAngleController::maxVerticalAccelCb(double _p)
{
  max_ver_acc_ = _p;
}

void AccelAngleController::maxAttitudeCb(double _p)
{
  max_attitude_ = st::deg2rad(_p);
}

void AccelAngleController::maxAttitudeRateCb(double _p)
{
  const auto max_atti_rate = st::deg2rad(_p);  // [rad/s]
  roll_filt_.setMaxVelocity(max_atti_rate);
  pitch_filt_.setMaxVelocity(max_atti_rate);
}

void AccelAngleController::maxHeadingRateCb(double _p)
{
  max_head_rate_ = st::deg2rad(_p);
}

void AccelAngleController::horizontalAccelExpoCb(double _p)
{
  hor_acc_expo_ = _p / kExpoScale;
}

void AccelAngleController::verticalAccelExpoCb(double _p)
{
  ver_acc_expo_ = _p / kExpoScale;
}

void AccelAngleController::attitudeExpoCb(double _p)
{
  atti_expo_ = _p / kExpoScale;
}

void AccelAngleController::headingExpoCb(double _p)
{
  head_expo_ = _p / kExpoScale;
}
}  // namespace rc
}  // namespace tobas

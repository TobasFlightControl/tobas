// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rc_teleop/pos_vel_acc_pitch_yaw.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas::rc
{
PosVelAccPitchYawController::PosVelAccPitchYawController()
{
}

bool PosVelAccPitchYawController::requireHorizontalPosition()
{
  return true;
}

bool PosVelAccPitchYawController::requireVerticalPosition()
{
  return true;
}

bool PosVelAccPitchYawController::requireAttitude()
{
  return true;
}

bool PosVelAccPitchYawController::requireHeading()
{
  return true;
}

void PosVelAccPitchYawController::initialize(BaseNode* _node, FlightMode _mode)
{
  _node->addDynamicDoubleParam(
    addMode("max_horizontal_velocity", _mode), &self::maxHorizontalVelocityCb, this, 0.5, 12, 0, 20, " m/s");
  _node->addDynamicDoubleParam(
    addMode("max_horizontal_jerk", _mode), &self::maxHorizontalJerkCb, this, 5.0, 8, 1, 20, " m/s^3");
  _node->addDynamicDoubleParam(
    addMode("max_vertical_velocity", _mode), &self::maxVerticalVelocityCb, this, 0.5, 8, 0, 20, " m/s");
  _node->addDynamicDoubleParam(
    addMode("max_vertical_jerk", _mode), &self::maxVerticalJerkCb, this, 5.0, 8, 1, 20, " m/s^3");
  _node->addDynamicDoubleParam(addMode("max_pitch", _mode), &self::maxPitchCb, this, 15.0, 6, 1, 12, " deg");
  _node->addDynamicDoubleParam(addMode("max_pitch_rate", _mode), &self::maxPitchRateCb, this, 15.0, 6, 1, 12, " dps");
  _node->addDynamicDoubleParam(addMode("max_yaw_rate", _mode), &self::maxYawRateCb, this, 15.0, 6, 1, 12, " dps");
  _node->addDynamicDoubleParam(
    addMode("max_position_error_down", _mode), &self::maxPositionErrorDown, this, 0.5, 4, 0, 20, " m");
  _node->addDynamicDoubleParam(
    addMode("horizontal_velocity_expo", _mode), &self::horizontalVelocityExpoCb, this, 5.0, -6, -20, 20);
  _node->addDynamicDoubleParam(
    addMode("vertical_velocity_expo", _mode), &self::verticalVelocityExpoCb, this, 5.0, 0, -20, 20);
  _node->addDynamicDoubleParam(addMode("pitch_expo", _mode), &self::pitchExpoCb, this, 5.0, 0, -20, 20);
  _node->addDynamicDoubleParam(addMode("yaw_expo", _mode), &self::yawExpoCb, this, 5.0, -3, -20, 20);

  cmd_pub_ = _node->createPublisher<tobas_command_msgs::PosVelAccPitchYaw>(topic::kPosVelAccPitchYawCmd);
}

void PosVelAccPitchYawController::reset(
  const builtin_interfaces::msg::Time& _stamp,
  const tobas_msgs::Odometry& _setpoint,
  bool _landed)
{
  t_last_rcin_ = _stamp;

  const auto [roll, pitch, yaw] = _setpoint.frame.M.getRPY();

  const auto R_G_B = kdl::Rotation::RPY(roll, pitch, 0.0);
  const auto tar_vel_G = R_G_B * _setpoint.twist.vel;
  vx_filt_.resetCurrentTrajectoryPoint(tar_vel_G.x(), 0.0);
  vy_filt_.resetCurrentTrajectoryPoint(tar_vel_G.y(), 0.0);
  vz_filt_.resetCurrentTrajectoryPoint(tar_vel_G.z(), 0.0);

  tar_pos_W_ = _setpoint.frame.p;

  if (_landed) {
    vz_filt_.resetCurrentTrajectoryPoint(-max_ver_vel_, 0.0);
    tar_pos_W_.z() -= max_ep_down_;
  }

  pitch_filt_.resetCurrentTrajectoryPoint(pitch);
  tar_yaw_ = _setpoint.frame.M.getYaw();
}

void PosVelAccPitchYawController::update(const tobas_msgs::RCInput& _rcin, const tobas_msgs::Odometry& _odom, bool _landed)
{
  // Update timestamp.
  const auto dt = (_rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = _rcin.header.stamp;

  // Velocity-X & Pitch
  if (_rcin.sub_mode)  // Translation mode
  {
    vx_filt_.update(expoRemapDead(_rcin.pitch, hor_vel_expo_, -max_hor_vel_, max_hor_vel_), dt);
    pitch_filt_.setTargetPointAndUpdate(0.0, dt);
  }
  else  // Rotation mode
  {
    vx_filt_.update(0.0, dt);
    pitch_filt_.setTargetPointAndUpdate(expoRemapDead(_rcin.pitch, pitch_expo_, -max_pitch_, max_pitch_), dt);
  }

  // Velocity-YZ
  vy_filt_.update(-expoRemap(_rcin.roll, hor_vel_expo_, -max_hor_vel_, max_hor_vel_), dt);
  vz_filt_.update(expoRemapDead(_rcin.throttle, ver_vel_expo_, -max_ver_vel_, max_ver_vel_), dt);

  // Yaw
  const auto yawrate = expoRemapDead(_rcin.yaw, yaw_expo_, -max_yaw_rate_, max_yaw_rate_);
  tar_yaw_ += yawrate * dt;

  // Compute the velocity and acceleration wrt. the world frame.
  const kdl::Vector tar_vel_G(
    vx_filt_.getTrajectoryVelocity(), vy_filt_.getTrajectoryVelocity(), vz_filt_.getTrajectoryVelocity());
  const kdl::Vector tar_acc_G(
    vx_filt_.getTrajectoryAccel(), vy_filt_.getTrajectoryAccel(), vz_filt_.getTrajectoryAccel());
  const auto R_W_G = kdl::Rotation::RotZ(tar_yaw_);
  const auto tar_vel_W = R_W_G * tar_vel_G;
  const auto tar_acc_W = R_W_G * tar_acc_G;

  // Integrate the velocity.
  tar_pos_W_ += tar_vel_W * dt;

  // Do not perform horizontal position control while landed.
  const auto& cur_pos_W = _odom.frame.p;
  if (_landed) {
    tar_pos_W_.x() = cur_pos_W.x();
    tar_pos_W_.y() = cur_pos_W.y();
  }

  // Limit the error to prevent the target altitude from dropping too far while on the ground.
  const auto& cur_z = cur_pos_W.z();
  tar_pos_W_.z() = std::max(tar_pos_W_.z(), cur_z - max_ep_down_);

  // Create a command.
  auto cmd = std::make_unique<tobas_command_msgs::PosVelAccPitchYaw>();
  cmd->header = _rcin.header;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
  cmd->pos = tar_pos_W_;
  cmd->vel = tar_vel_W;
  cmd->acc = tar_acc_W;
  cmd->pitch = pitch_filt_.getTrajectoryPosition();
  cmd->yaw = tar_yaw_;

  // Publish the command.
  cmd_pub_->publish(std::move(cmd));
}

void PosVelAccPitchYawController::maxHorizontalVelocityCb(double _p)
{
  max_hor_vel_ = _p;
  vx_filt_.setMaxVelocity(_p);
  vy_filt_.setMaxVelocity(_p);
}

void PosVelAccPitchYawController::maxHorizontalJerkCb(double _p)
{
  vx_filt_.setMaxJerk(_p);
  vy_filt_.setMaxJerk(_p);
}

void PosVelAccPitchYawController::maxVerticalVelocityCb(double _p)
{
  max_ver_vel_ = _p;
  vz_filt_.setMaxVelocity(_p);
}

void PosVelAccPitchYawController::maxVerticalJerkCb(double _p)
{
  vz_filt_.setMaxJerk(_p);
}

void PosVelAccPitchYawController::maxPitchCb(double _p)
{
  max_pitch_ = st::deg2rad(_p);
}

void PosVelAccPitchYawController::maxPitchRateCb(double _p)
{
  pitch_filt_.setMaxVelocity(st::deg2rad(_p));
}

void PosVelAccPitchYawController::maxYawRateCb(double _p)
{
  max_yaw_rate_ = st::deg2rad(_p);
}

void PosVelAccPitchYawController::maxPositionErrorDown(double _p)
{
  max_ep_down_ = _p;
}

void PosVelAccPitchYawController::horizontalVelocityExpoCb(double _p)
{
  hor_vel_expo_ = _p / kExpoScale;
}

void PosVelAccPitchYawController::verticalVelocityExpoCb(double _p)
{
  ver_vel_expo_ = _p / kExpoScale;
}

void PosVelAccPitchYawController::pitchExpoCb(double _p)
{
  pitch_expo_ = _p / kExpoScale;
}

void PosVelAccPitchYawController::yawExpoCb(double _p)
{
  yaw_expo_ = _p / kExpoScale;
}
}  // namespace tobas::rc

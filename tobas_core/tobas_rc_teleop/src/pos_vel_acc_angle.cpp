// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rc_teleop/pos_vel_acc_angle.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace rc
{
PosVelAccAngleController::PosVelAccAngleController()
{
}

bool PosVelAccAngleController::requirePosition()
{
  return true;
}

bool PosVelAccAngleController::requireVelocity()
{
  return true;
}

bool PosVelAccAngleController::requireAttitude()
{
  return true;
}

bool PosVelAccAngleController::requireHeading()
{
  return true;
}

void PosVelAccAngleController::initialize(BaseNode* node, FlightMode mode)
{
  node->addDynamicDoubleParam(
    addMode("max_horizontal_velocity", mode), &self::maxHorizontalVelocityCb, this, 0.5, 12, 0, 20, " m/s");
  node->addDynamicDoubleParam(
    addMode("max_horizontal_accel", mode), &self::maxHorizontalAccelCb, this, 1., 10, 1, 20, " m/s^2");
  node->addDynamicDoubleParam(
    addMode("max_vertical_velocity", mode), &self::maxVerticalVelocityCb, this, 0.5, 8, 0, 20, " m/s");
  node->addDynamicDoubleParam(
    addMode("max_vertical_accel", mode), &self::maxVerticalAccelCb, this, 1., 10, 1, 20, " m/s^2");
  node->addDynamicDoubleParam(addMode("max_attitude", mode), &self::maxAttitudeCb, this, 10., 9, 1, 18, " deg");
  node->addDynamicDoubleParam(addMode("max_attitude_rate", mode), &self::maxAttitudeRateCb, this, 20., 9, 1, 18, " dps");
  node->addDynamicDoubleParam(addMode("max_heading_rate", mode), &self::maxHeadingRateCb, this, 20., 9, 1, 18, " dps");
  node->addDynamicDoubleParam(
    addMode("max_position_error_down", mode), &self::maxPositionErrorDown, this, 0.5, 4, 0, 20, " m");
  node->addDynamicDoubleParam(
    addMode("horizontal_velocity_expo", mode), &self::horizontalVelocityExpoCb, this, 5., -6, -20, 20);
  node->addDynamicDoubleParam(
    addMode("vertical_velocity_expo", mode), &self::verticalVelocityExpoCb, this, 5., 0, -20, 20);
  node->addDynamicDoubleParam(addMode("attitude_expo", mode), &self::attitudeExpoCb, this, 5., 0, -20, 20);
  node->addDynamicDoubleParam(addMode("heading_expo", mode), &self::headingExpoCb, this, 5., -3, -20, 20);

  pos_vel_acc_pub_ = node->createPublisher<tobas_command_msgs::PosVelAcc>(topic::kPosVelAccCmd);
  angle_pub_ = node->createPublisher<tobas_command_msgs::Angle>(topic::kAngleCmd);
}

void PosVelAccAngleController::reset(
  const builtin_interfaces::msg::Time& stamp,
  const tobas_msgs::Odometry& setpoint,
  bool landed)
{
  t_last_rcin_ = stamp;

  const auto [roll, pitch, yaw] = setpoint.frame.M.getRPY();

  const auto R_G_B = kdl::Rotation::RPY(roll, pitch, 0.);
  const auto tar_vel_G = R_G_B * setpoint.twist.vel;
  vx_filt_.resetCurrentTrajectoryPoint(tar_vel_G.x());
  vy_filt_.resetCurrentTrajectoryPoint(tar_vel_G.y());
  vz_filt_.resetCurrentTrajectoryPoint(tar_vel_G.z());

  tar_pos_W_ = setpoint.frame.p;

  if (landed) {
    vz_filt_.resetCurrentTrajectoryPoint(-max_ver_vel_);
    tar_pos_W_.z() -= max_ep_down_;
  }

  roll_filt_.resetCurrentTrajectoryPoint(roll);
  pitch_filt_.resetCurrentTrajectoryPoint(pitch);
  tar_yaw_ = yaw;
}

void PosVelAccAngleController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed)
{
  // Update timestamp
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // Horizontal velocity & Attitude
  if (rcin.sub_mode)  // Translation mode
  {
    vx_filt_.setTargetPointAndUpdate(expoRemap(rcin.pitch, hor_vel_expo_, -max_hor_vel_, max_hor_vel_), dt);
    vy_filt_.setTargetPointAndUpdate(-expoRemap(rcin.roll, hor_vel_expo_, -max_hor_vel_, max_hor_vel_), dt);
    roll_filt_.setTargetPointAndUpdate(0., dt);
    pitch_filt_.setTargetPointAndUpdate(0., dt);
  }
  else  // Rotation mode
  {
    roll_filt_.setTargetPointAndUpdate(expoRemapDead(rcin.roll, atti_expo_, -max_attitude_, max_attitude_), dt);
    pitch_filt_.setTargetPointAndUpdate(expoRemapDead(rcin.pitch, atti_expo_, -max_attitude_, max_attitude_), dt);
    vx_filt_.setTargetPointAndUpdate(0., dt);
    vy_filt_.setTargetPointAndUpdate(0., dt);
  }

  // Vertical velocity
  vz_filt_.setTargetPointAndUpdate(expoRemapDead(rcin.throttle, ver_vel_expo_, -max_ver_vel_, max_ver_vel_), dt);

  // Yaw
  const auto yawrate = expoRemapDead(rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_);
  tar_yaw_ += yawrate * dt;

  // Compute the velocity wrt. the world frame
  const kdl::Vector tar_vel_G(
    vx_filt_.getTrajectoryPosition(), vy_filt_.getTrajectoryPosition(), vz_filt_.getTrajectoryPosition());
  const auto tar_vel_W = kdl::Rotation::RotZ(tar_yaw_) * tar_vel_G;

  // Integrate the velocity
  tar_pos_W_ += tar_vel_W * dt;

  // Do not perform horizontal position control while landed
  const auto& cur_pos_W = odom.frame.p;
  if (landed) {
    tar_pos_W_.x() = cur_pos_W.x();
    tar_pos_W_.y() = cur_pos_W.y();
  }

  // Limit the error to prevent the target altitude from dropping too far while on the ground
  const auto& cur_z = cur_pos_W.z();
  tar_pos_W_.z() = std::max(tar_pos_W_.z(), cur_z - max_ep_down_);

  // Publish commands
  publishPosVelAcc(rcin.header.stamp, tar_pos_W_, tar_vel_W, kdl::Vector::Zero());
  publishAngle(rcin.header.stamp, roll_filt_.getTrajectoryPosition(), pitch_filt_.getTrajectoryPosition(), tar_yaw_);
}

void PosVelAccAngleController::publishPosVelAcc(
  const builtin_interfaces::msg::Time& stamp,
  const kdl::Vector& pos,
  const kdl::Vector& vel,
  const kdl::Vector& acc)
{
  auto cmd = std::make_unique<tobas_command_msgs::PosVelAcc>();
  cmd->header.stamp = stamp;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
  cmd->pos = pos;
  cmd->vel = vel;
  cmd->acc = acc;

  pos_vel_acc_pub_->publish(std::move(cmd));
}

void PosVelAccAngleController::publishAngle(
  const builtin_interfaces::msg::Time& stamp,
  double roll,
  double pitch,
  double yaw)
{
  auto cmd = std::make_unique<tobas_command_msgs::Angle>();
  cmd->header.stamp = stamp;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
  cmd->angle.set(roll, pitch, yaw);

  angle_pub_->publish(std::move(cmd));
}

bool PosVelAccAngleController::maxHorizontalVelocityCb(const double& p)
{
  max_hor_vel_ = p;
  return true;
}

bool PosVelAccAngleController::maxHorizontalAccelCb(const double& p)
{
  vx_filt_.setMaxVelocity(p);
  vy_filt_.setMaxVelocity(p);
  return true;
}

bool PosVelAccAngleController::maxVerticalVelocityCb(const double& p)
{
  max_ver_vel_ = p;
  return true;
}

bool PosVelAccAngleController::maxVerticalAccelCb(const double& p)
{
  vz_filt_.setMaxVelocity(p);
  return true;
}

bool PosVelAccAngleController::maxAttitudeCb(const double& p)
{
  max_attitude_ = st::deg2rad(p);
  return true;
}

bool PosVelAccAngleController::maxAttitudeRateCb(const double& p)
{
  const auto max_atti_rate = st::deg2rad(p);  // [rad/s]
  roll_filt_.setMaxVelocity(max_atti_rate);
  pitch_filt_.setMaxVelocity(max_atti_rate);
  return true;
}

bool PosVelAccAngleController::maxHeadingRateCb(const double& p)
{
  max_head_rate_ = st::deg2rad(p);
  return true;
}

bool PosVelAccAngleController::maxPositionErrorDown(const double& p)
{
  max_ep_down_ = p;
  return true;
}

bool PosVelAccAngleController::horizontalVelocityExpoCb(const double& p)
{
  hor_vel_expo_ = p / kExpoScale;
  return true;
}

bool PosVelAccAngleController::verticalVelocityExpoCb(const double& p)
{
  ver_vel_expo_ = p / kExpoScale;
  return true;
}

bool PosVelAccAngleController::attitudeExpoCb(const double& p)
{
  atti_expo_ = p / kExpoScale;
  return true;
}

bool PosVelAccAngleController::headingExpoCb(const double& p)
{
  head_expo_ = p / kExpoScale;
  return true;
}
}  // namespace rc
}  // namespace tobas

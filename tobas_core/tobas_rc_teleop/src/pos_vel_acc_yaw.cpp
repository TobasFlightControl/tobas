// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rc_teleop/pos_vel_acc_yaw.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace rc
{
PosVelAccYawController::PosVelAccYawController()
{
}

bool PosVelAccYawController::requirePosition()
{
  return true;
}

bool PosVelAccYawController::requireVelocity()
{
  return true;
}

bool PosVelAccYawController::requireAttitude()
{
  return false;
}

bool PosVelAccYawController::requireHeading()
{
  return false;
}

void PosVelAccYawController::initialize(BaseNode* node, FlightMode mode)
{
  node->addDynamicDoubleParam(
    addMode("max_horizontal_velocity", mode), &self::maxHorizontalVelocityCb, this, 0.5, 12, 0, 20, " m/s");
  node->addDynamicDoubleParam(
    addMode("max_horizontal_jerk", mode), &self::maxHorizontalJerkCb, this, 5., 8, 1, 20, " m/s^3");
  node->addDynamicDoubleParam(
    addMode("max_vertical_velocity", mode), &self::maxVerticalVelocityCb, this, 0.5, 8, 0, 20, " m/s");
  node->addDynamicDoubleParam(
    addMode("max_vertical_jerk", mode), &self::maxVerticalJerkCb, this, 5., 8, 1, 20, " m/s^3");
  node->addDynamicDoubleParam(addMode("max_heading_rate", mode), &self::maxHeadingRateCb, this, 15., 6, 1, 12, " dps");
  node->addDynamicDoubleParam(
    addMode("max_position_error_down", mode), &self::maxPositionErrorDown, this, 0.5, 4, 0, 20, " m");
  node->addDynamicDoubleParam(
    addMode("horizontal_velocity_expo", mode), &self::horizontalVelocityExpoCb, this, 5., -6, -20, 20);
  node->addDynamicDoubleParam(
    addMode("vertical_velocity_expo", mode), &self::verticalVelocityExpoCb, this, 5., 0, -20, 20);
  node->addDynamicDoubleParam(addMode("heading_expo", mode), &self::headingExpoCb, this, 5., -3, -20, 20);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::PosVelAccYaw>(topic::kPosVelAccYawCmd);
}

void PosVelAccYawController::reset(
  const builtin_interfaces::msg::Time& stamp,
  const tobas_msgs::Odometry& setpoint,
  bool landed)
{
  t_last_rcin_ = stamp;

  const auto [roll, pitch, yaw] = setpoint.frame.M.getRPY();

  const auto R_G_B = kdl::Rotation::RPY(roll, pitch, 0.);
  const auto tar_vel_G = R_G_B * setpoint.twist.vel;
  vx_filt_.resetCurrentTrajectoryPoint(tar_vel_G.x(), 0.);
  vy_filt_.resetCurrentTrajectoryPoint(tar_vel_G.y(), 0.);
  vz_filt_.resetCurrentTrajectoryPoint(tar_vel_G.z(), 0.);

  tar_pos_W_ = setpoint.frame.p;

  if (landed) {
    vz_filt_.resetCurrentTrajectoryPoint(-max_ver_vel_, 0.);
    tar_pos_W_.z() -= max_ep_down_;
  }

  tar_yaw_ = yaw;
}

void PosVelAccYawController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed)
{
  // Update time
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // Velocity
  vx_filt_.update(expoRemapDead(rcin.pitch, hor_vel_expo_, -max_hor_vel_, max_hor_vel_), dt);
  vy_filt_.update(-expoRemapDead(rcin.roll, hor_vel_expo_, -max_hor_vel_, max_hor_vel_), dt);
  vz_filt_.update(expoRemapDead(rcin.throttle, ver_vel_expo_, -max_ver_vel_, max_ver_vel_), dt);

  // Yaw
  const auto yawrate = expoRemapDead(rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_);
  tar_yaw_ += yawrate * dt;

  // Compute the velocity and acceleration wrt. the world frame
  const kdl::Vector tar_vel_G(
    vx_filt_.getTrajectoryVelocity(), vy_filt_.getTrajectoryVelocity(), vz_filt_.getTrajectoryVelocity());
  const kdl::Vector tar_acc_G(
    vx_filt_.getTrajectoryAccel(), vy_filt_.getTrajectoryAccel(), vz_filt_.getTrajectoryAccel());
  const auto R_W_G = kdl::Rotation::RotZ(tar_yaw_);
  const auto tar_vel_W = R_W_G * tar_vel_G;
  const auto tar_acc_W = R_W_G * tar_acc_G;

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

  // Create a command
  auto cmd = std::make_unique<tobas_command_msgs::PosVelAccYaw>();
  cmd->header = rcin.header;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
  cmd->pos = tar_pos_W_;
  cmd->vel = tar_vel_W;
  cmd->acc = tar_acc_W;
  cmd->yaw = tar_yaw_;

  // Publish the command
  cmd_pub_->publish(std::move(cmd));
}

bool PosVelAccYawController::maxHorizontalVelocityCb(const double& p)
{
  max_hor_vel_ = p;
  vx_filt_.setMaxVelocity(p);
  vy_filt_.setMaxVelocity(p);
  return true;
}

bool PosVelAccYawController::maxHorizontalJerkCb(const double& p)
{
  vx_filt_.setMaxJerk(p);
  vy_filt_.setMaxJerk(p);
  return true;
}

bool PosVelAccYawController::maxVerticalVelocityCb(const double& p)
{
  max_ver_vel_ = p;
  vz_filt_.setMaxVelocity(p);
  return true;
}

bool PosVelAccYawController::maxVerticalJerkCb(const double& p)
{
  vz_filt_.setMaxJerk(p);
  return true;
}

bool PosVelAccYawController::maxHeadingRateCb(const double& p)
{
  max_head_rate_ = st::deg2rad(p);
  return true;
}

bool PosVelAccYawController::maxPositionErrorDown(const double& p)
{
  max_ep_down_ = p;
  return true;
}

bool PosVelAccYawController::horizontalVelocityExpoCb(const double& p)
{
  hor_vel_expo_ = p / kExpoScale;
  return true;
}

bool PosVelAccYawController::verticalVelocityExpoCb(const double& p)
{
  ver_vel_expo_ = p / kExpoScale;
  return true;
}

bool PosVelAccYawController::headingExpoCb(const double& p)
{
  head_expo_ = p / kExpoScale;
  return true;
}
}  // namespace rc
}  // namespace tobas

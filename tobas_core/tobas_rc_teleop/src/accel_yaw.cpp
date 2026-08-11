// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rc_teleop/accel_yaw.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace rc
{
AccelYawController::AccelYawController()
{
}

bool AccelYawController::requireHorizontalPosition()
{
  return false;
}

bool AccelYawController::requireVerticalPosition()
{
  return false;
}

bool AccelYawController::requireAttitude()
{
  return true;
}

bool AccelYawController::requireHeading()
{
  return true;
}

void AccelYawController::initialize(BaseNode* node, FlightMode mode)
{
  node->addDynamicDoubleParam(
    addMode("max_horizontal_accel", mode), &self::maxHorizontalAccelCb, this, 0.5, 10, 1, 20, " m/s^2");
  node->addDynamicDoubleParam(
    addMode("max_horizontal_jerk", mode), &self::maxHorizontalJerkCb, this, 5.0, 8, 1, 20, " m/s^3");
  node->addDynamicDoubleParam(
    addMode("max_vertical_accel", mode), &self::maxVerticalAccelCb, this, 0.5, 16, 1, 20, " m/s^2");
  node->addDynamicDoubleParam(addMode("max_heading_rate", mode), &self::maxHeadingRateCb, this, 15.0, 6, 1, 12, " dps");
  node->addDynamicDoubleParam(
    addMode("horizontal_accel_expo", mode), &self::horizontalAccelExpoCb, this, 5.0, -6, -20, 20);
  node->addDynamicDoubleParam(addMode("vertical_accel_expo", mode), &self::verticalAccelExpoCb, this, 5.0, 0, -20, 20);
  node->addDynamicDoubleParam(addMode("heading_expo", mode), &self::headingExpoCb, this, 5.0, -3, -20, 20);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::AccelYaw>(topic::kAccelYawCmd);
}

void AccelYawController::reset(const builtin_interfaces::msg::Time& stamp, const tobas_msgs::Odometry& setpoint, bool)
{
  t_last_rcin_ = stamp;

  ax_filt_.resetCurrentTrajectoryPoint(0.0);
  ay_filt_.resetCurrentTrajectoryPoint(0.0);

  tar_yaw_ = setpoint.frame.M.getYaw();
}

void AccelYawController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry&, bool)
{
  // Update timestamp.
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // Horizontal acceleration
  ax_filt_.setTargetPointAndUpdate(expoRemap(rcin.pitch, hor_acc_expo_, -max_hor_acc_, max_hor_acc_), dt);
  ay_filt_.setTargetPointAndUpdate(-expoRemap(rcin.roll, hor_acc_expo_, -max_hor_acc_, max_hor_acc_), dt);

  // Vertical acceleration
  const auto az = expoRemap(rcin.throttle, ver_acc_expo_, -max_ver_acc_, max_ver_acc_);

  // Yaw
  const auto yawrate = expoRemapDead(rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_);
  tar_yaw_ += yawrate * dt;

  // Compute the acceleration wrt. the world frame.
  const kdl::Vector tar_acc_G(ax_filt_.getTrajectoryPosition(), ay_filt_.getTrajectoryPosition(), az);
  const auto tar_acc_W = kdl::Rotation::RotZ(tar_yaw_) * tar_acc_G;

  // Create a command.
  auto cmd = std::make_unique<tobas_command_msgs::AccelYaw>();
  cmd->header = rcin.header;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
  cmd->accel = tar_acc_W;
  cmd->yaw = tar_yaw_;

  // Publish the command.
  cmd_pub_->publish(std::move(cmd));
}

bool AccelYawController::maxHorizontalAccelCb(const double& p)
{
  max_hor_acc_ = p;
  return true;
}

bool AccelYawController::maxHorizontalJerkCb(const double& p)
{
  ax_filt_.setMaxVelocity(p);
  ay_filt_.setMaxVelocity(p);
  return true;
}

bool AccelYawController::maxVerticalAccelCb(const double& p)
{
  max_ver_acc_ = p;
  return true;
}

bool AccelYawController::maxHeadingRateCb(const double& p)
{
  max_head_rate_ = st::deg2rad(p);
  return true;
}

bool AccelYawController::horizontalAccelExpoCb(const double& p)
{
  hor_acc_expo_ = p / kExpoScale;
  return true;
}

bool AccelYawController::verticalAccelExpoCb(const double& p)
{
  ver_acc_expo_ = p / kExpoScale;
  return true;
}

bool AccelYawController::headingExpoCb(const double& p)
{
  head_expo_ = p / kExpoScale;
  return true;
}
}  // namespace rc
}  // namespace tobas

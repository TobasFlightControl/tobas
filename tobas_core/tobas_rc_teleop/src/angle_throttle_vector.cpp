// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rc_teleop/angle_throttle_vector.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_constants/throttle.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace rc
{
AngleThrottleVectorController::AngleThrottleVectorController()
{
}

bool AngleThrottleVectorController::requirePosition()
{
  return false;
}

bool AngleThrottleVectorController::requireVelocity()
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

void AngleThrottleVectorController::initialize(BaseNode* node, FlightMode mode)
{
  node->addDynamicDoubleParam(addMode("max_roll", mode), &self::maxRollCb, this, 5.0, 9, 1, 16, " deg");
  node->addDynamicDoubleParam(addMode("max_roll_rate", mode), &self::maxRollRateCb, this, 15.0, 6, 1, 12, " dps");
  node->addDynamicDoubleParam(addMode("max_pitch", mode), &self::maxPitchCb, this, 15.0, 6, 1, 12, " deg");
  node->addDynamicDoubleParam(addMode("max_pitch_rate", mode), &self::maxPitchRateCb, this, 15.0, 6, 1, 12, " dps");
  node->addDynamicDoubleParam(addMode("max_yaw_rate", mode), &self::maxYawRateCb, this, 15.0, 6, 1, 12, " dps");
  node->addDynamicDoubleParam(addMode("max_thrust_angle", mode), &self::maxThrustAngleCb, this, 15.0, 6, 1, 12, " deg");
  node->addDynamicDoubleParam(
    addMode("max_thrust_angle_rate", mode), &self::maxThrustAngleRateCb, this, 15.0, 6, 1, 12, " dps");
  node->addDynamicDoubleParam(addMode("roll_expo", mode), &self::rollExpoCb, this, 5.0, -6, -20, 20);
  node->addDynamicDoubleParam(addMode("yaw_expo", mode), &self::yawExpoCb, this, 5.0, -3, -20, 20);
  node->addDynamicDoubleParam(addMode("throttle_expo", mode), &self::throttleExpoCb, this, 5.0, 0, 0, 20);
  node->addDynamicDoubleParam(addMode("thrust_angle_expo", mode), &self::thrustAngleExpoCb, this, 5.0, 0, -20, 20);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::AngleThrottleVector>(topic::kAngleThrotVectorCmd);
}

void AngleThrottleVectorController::reset(
  const builtin_interfaces::msg::Time& stamp,
  const tobas_msgs::Odometry& setpoint,
  bool)
{
  t_last_rcin_ = stamp;

  const auto [roll, pitch, yaw] = setpoint.frame.M.getRPY();
  roll_filt_.resetCurrentTrajectoryPoint(roll);
  pitch_filt_.resetCurrentTrajectoryPoint(pitch);
  thrust_angle_filt_.resetCurrentTrajectoryPoint(-pitch);
  tar_yaw_ = yaw;
}

void AngleThrottleVectorController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry&, bool)
{
  // Update timestamp
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // Create a command
  auto cmd = std::make_unique<tobas_command_msgs::AngleThrottleVector>();
  cmd->header = rcin.header;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;

  // Roll
  roll_filt_.setTargetPointAndUpdate(expoRemapDead(rcin.roll, roll_expo_, -max_roll_, max_roll_), dt);
  cmd->angle.roll = roll_filt_.getTrajectoryPosition();

  // Yaw
  const auto yawrate = expoRemapDead(rcin.yaw, yaw_expo_, -max_yaw_rate_, max_yaw_rate_);
  tar_yaw_ += yawrate * dt;
  cmd->angle.yaw = tar_yaw_;

  // Throttle
  cmd->throttle = expo(remap(rcin.throttle, kMinThrot, kMaxThrot), throt_expo_);

  // Pitch & Thrust Angle
  if (rcin.sub_mode)  // Translation mode
  {
    pitch_filt_.setTargetPointAndUpdate(0.0, dt);
    thrust_angle_filt_.setTargetPointAndUpdate(
      expoRemapDead(rcin.pitch, thrust_angle_expo_, -max_thrust_angle_, max_thrust_angle_), dt);
  }
  else  // Rotation mode
  {
    const auto tar_pitch = remapDead(rcin.pitch, -max_pitch_, max_pitch_);
    pitch_filt_.setTargetPointAndUpdate(tar_pitch, dt);
    thrust_angle_filt_.setTargetPointAndUpdate(-tar_pitch, dt);
  }
  cmd->angle.pitch = pitch_filt_.getTrajectoryPosition();
  cmd->thrust_angle = thrust_angle_filt_.getTrajectoryPosition();

  // Publish the command
  cmd_pub_->publish(std::move(cmd));
}

bool AngleThrottleVectorController::maxRollCb(const double& p)
{
  max_roll_ = st::deg2rad(p);
  return true;
}

bool AngleThrottleVectorController::maxRollRateCb(const double& p)
{
  roll_filt_.setMaxVelocity(st::deg2rad(p));
  return true;
}

bool AngleThrottleVectorController::maxPitchCb(const double& p)
{
  max_pitch_ = st::deg2rad(p);
  return true;
}

bool AngleThrottleVectorController::maxPitchRateCb(const double& p)
{
  pitch_filt_.setMaxVelocity(st::deg2rad(p));
  return true;
}

bool AngleThrottleVectorController::maxYawRateCb(const double& p)
{
  max_yaw_rate_ = st::deg2rad(p);
  return true;
}

bool AngleThrottleVectorController::maxThrustAngleCb(const double& p)
{
  max_thrust_angle_ = st::deg2rad(p);
  return true;
}

bool AngleThrottleVectorController::maxThrustAngleRateCb(const double& p)
{
  thrust_angle_filt_.setMaxVelocity(st::deg2rad(p));
  return true;
}

bool AngleThrottleVectorController::rollExpoCb(const double& p)
{
  roll_expo_ = p / kExpoScale;
  return true;
}

bool AngleThrottleVectorController::yawExpoCb(const double& p)
{
  yaw_expo_ = p / kExpoScale;
  return true;
}

bool AngleThrottleVectorController::throttleExpoCb(const double& p)
{
  throt_expo_ = p / kExpoScale;
  return true;
}

bool AngleThrottleVectorController::thrustAngleExpoCb(const double& p)
{
  thrust_angle_expo_ = p / kExpoScale;
  return true;
}
}  // namespace rc
}  // namespace tobas

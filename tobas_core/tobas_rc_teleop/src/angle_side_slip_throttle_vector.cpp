// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rc_teleop/angle_side_slip_throttle_vector.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_constants/throttle.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace rc
{
AngleSideSlipThrottleVectorController::AngleSideSlipThrottleVectorController()
{
}

bool AngleSideSlipThrottleVectorController::requireHorizontalPosition()
{
  return false;
}

bool AngleSideSlipThrottleVectorController::requireVerticalPosition()
{
  return false;
}

bool AngleSideSlipThrottleVectorController::requireAttitude()
{
  return true;
}

bool AngleSideSlipThrottleVectorController::requireHeading()
{
  return true;
}

void AngleSideSlipThrottleVectorController::initialize(BaseNode* node, FlightMode mode)
{
  node->addDynamicDoubleParam(addMode("max_roll", mode), &self::maxRollCb, this, 5.0, 9, 1, 16, " deg");
  node->addDynamicDoubleParam(addMode("max_roll_rate", mode), &self::maxRollRateCb, this, 15.0, 6, 1, 12, " dps");
  node->addDynamicDoubleParam(addMode("max_pitch", mode), &self::maxPitchCb, this, 2.0, 10, 1, 15, " deg");
  node->addDynamicDoubleParam(addMode("max_pitch_rate", mode), &self::maxPitchRateCb, this, 5.0, 6, 1, 12, " dps");
  node->addDynamicDoubleParam(addMode("max_side_slip", mode), &self::maxSideSlipCb, this, 1.0, 6, 1, 12, " deg");
  node->addDynamicDoubleParam(addMode("max_yaw_rate", mode), &self::maxYawRateCb, this, 15.0, 6, 1, 12, " dps");
  node->addDynamicDoubleParam(
    addMode("max_thrust_angle_rate", mode), &self::maxThrustAngleRateCb, this, 5.0, 6, 1, 12, " dps");
  node->addDynamicDoubleParam(addMode("roll_expo", mode), &self::rollExpoCb, this, 5.0, -6, -20, 20);
  node->addDynamicDoubleParam(addMode("pitch_expo", mode), &self::pitchExpoCb, this, 5.0, -6, -20, 20);
  node->addDynamicDoubleParam(addMode("yaw_expo", mode), &self::yawExpoCb, this, 5.0, -3, -20, 20);
  node->addDynamicDoubleParam(addMode("throttle_expo", mode), &self::throttleExpoCb, this, 5.0, 0, 0, 20);

  multicopter_cmd_pub_ = node->createPublisher<tobas_command_msgs::AngleThrottleVector>(topic::kAngleThrotVectorCmd);
  fixed_wing_cmd_pub_ = node->createPublisher<tobas_command_msgs::msg::RollPitchSideSlipThrottleVector>(
    topic::kRollPitchSideSlipThrotVectorCmd);
}

void AngleSideSlipThrottleVectorController::reset(
  const builtin_interfaces::msg::Time& stamp,
  const tobas_msgs::Odometry& setpoint,
  bool)
{
  t_last_rcin_ = stamp;

  const auto [roll, pitch, yaw] = setpoint.frame.M.getRPY();
  roll_filt_.resetCurrentTrajectoryPoint(roll);
  pitch_filt_.resetCurrentTrajectoryPoint(pitch);
  // TODO: thrust_angle_filt_を現在のthrust_angleに設定
  tar_yaw_ = yaw;
}

void AngleSideSlipThrottleVectorController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool)
{
  // Update timestamp.
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // Roll
  roll_filt_.setTargetPointAndUpdate(expoRemapDead(rcin.roll, roll_expo_, -max_roll_, max_roll_), dt);
  const auto roll = roll_filt_.getTrajectoryPosition();

  // Pitch
  pitch_filt_.setTargetPointAndUpdate(expoRemapDead(rcin.pitch, pitch_expo_, -max_pitch_, max_pitch_), dt);
  const auto pitch = pitch_filt_.getTrajectoryPosition();

  // Throttle
  const auto throttle = expo(remap(rcin.throttle, kMinThrot, kMaxThrot), throt_expo_);

  // Tilt
  thrust_angle_filt_.setTargetPointAndUpdate(remap(rcin.vr, kMinThrustAngle, kMaxThrustAngle), dt);
  const auto thrust_angle = thrust_angle_filt_.getTrajectoryPosition();

  // Pitch & Thrust Angle
  if (thrust_angle < kModeChangeThrustAngle) {
    if (last_control_type_ == ControlType::FIXED_WING) {
      const auto [cur_roll, cur_pitch, cur_yaw] = odom.frame.M.getRPY();
      tar_yaw_ = cur_yaw;
    }
    last_control_type_ = ControlType::MULTICOPTER;
    // Yaw
    const auto yawrate = expoRemapDead(rcin.yaw, yaw_expo_, -max_yaw_rate_, max_yaw_rate_);
    tar_yaw_ += yawrate * dt;

    auto cmd = std::make_unique<tobas_command_msgs::AngleThrottleVector>();
    cmd->header = rcin.header;
    cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
    cmd->angle.roll = roll;
    cmd->angle.pitch = pitch;
    cmd->angle.yaw = tar_yaw_;
    cmd->throttle = throttle;
    cmd->thrust_angle = thrust_angle;
    multicopter_cmd_pub_->publish(std::move(cmd));
  }
  else {
    last_control_type_ = ControlType::FIXED_WING;
    auto cmd = std::make_unique<tobas_command_msgs::msg::RollPitchSideSlipThrottleVector>();
    cmd->header = rcin.header;
    cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
    cmd->roll = roll;
    cmd->pitch = pitch;
    cmd->side_slip = expoRemapDead(rcin.yaw, yaw_expo_, -max_side_slip_, max_side_slip_);
    cmd->throttle = throttle;
    cmd->thrust_angle = thrust_angle;
    fixed_wing_cmd_pub_->publish(std::move(cmd));
  }
}

bool AngleSideSlipThrottleVectorController::maxRollCb(const double& p)
{
  max_roll_ = st::deg2rad(p);
  return true;
}

bool AngleSideSlipThrottleVectorController::maxRollRateCb(const double& p)
{
  roll_filt_.setMaxVelocity(st::deg2rad(p));
  return true;
}

bool AngleSideSlipThrottleVectorController::maxPitchCb(const double& p)
{
  max_pitch_ = st::deg2rad(p);
  return true;
}

bool AngleSideSlipThrottleVectorController::maxPitchRateCb(const double& p)
{
  pitch_filt_.setMaxVelocity(st::deg2rad(p));
  return true;
}

bool AngleSideSlipThrottleVectorController::maxSideSlipCb(const double& p)
{
  max_side_slip_ = st::deg2rad(p);
  return true;
}

bool AngleSideSlipThrottleVectorController::maxYawRateCb(const double& p)
{
  max_yaw_rate_ = st::deg2rad(p);
  return true;
}

bool AngleSideSlipThrottleVectorController::maxThrustAngleRateCb(const double& p)
{
  thrust_angle_filt_.setMaxVelocity(st::deg2rad(p));
  return true;
}

bool AngleSideSlipThrottleVectorController::rollExpoCb(const double& p)
{
  roll_expo_ = p / kExpoScale;
  return true;
}

bool AngleSideSlipThrottleVectorController::pitchExpoCb(const double& p)
{
  pitch_expo_ = p / kExpoScale;
  return true;
}

bool AngleSideSlipThrottleVectorController::yawExpoCb(const double& p)
{
  yaw_expo_ = p / kExpoScale;
  return true;
}

bool AngleSideSlipThrottleVectorController::throttleExpoCb(const double& p)
{
  throt_expo_ = p / kExpoScale;
  return true;
}
}  // namespace rc
}  // namespace tobas

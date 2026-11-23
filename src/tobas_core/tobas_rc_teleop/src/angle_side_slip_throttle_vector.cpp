// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rc_teleop/angle_side_slip_throttle_vector.hpp"

#include <tobas_constants/ros.hpp>
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

bool AngleSideSlipThrottleVectorController::requireHorizontalPosition() const
{
  return false;
}

bool AngleSideSlipThrottleVectorController::requireVerticalPosition() const
{
  return false;
}

bool AngleSideSlipThrottleVectorController::requireAttitude() const
{
  return true;
}

bool AngleSideSlipThrottleVectorController::requireHeading() const
{
  return true;
}

void AngleSideSlipThrottleVectorController::initialize(BaseNode* _node, FlightMode _mode)
{
  _node->addDynamicDoubleParam(addMode("max_roll", _mode), &self::maxRollCb, this, 5.0, 9, 1, 16, " deg");
  _node->addDynamicDoubleParam(addMode("max_roll_rate", _mode), &self::maxRollRateCb, this, 15.0, 6, 1, 12, " dps");
  _node->addDynamicDoubleParam(addMode("max_pitch", _mode), &self::maxPitchCb, this, 2.0, 10, 1, 15, " deg");
  _node->addDynamicDoubleParam(addMode("max_pitch_rate", _mode), &self::maxPitchRateCb, this, 5.0, 6, 1, 12, " dps");
  _node->addDynamicDoubleParam(addMode("max_side_slip", _mode), &self::maxSideSlipCb, this, 1.0, 6, 1, 12, " deg");
  _node->addDynamicDoubleParam(addMode("max_yaw_rate", _mode), &self::maxYawRateCb, this, 15.0, 6, 1, 12, " dps");
  _node->addDynamicDoubleParam(
    addMode("max_thrust_angle_rate", _mode), &self::maxThrustAngleRateCb, this, 5.0, 6, 1, 12, " dps");
  _node->addDynamicDoubleParam(addMode("roll_expo", _mode), &self::rollExpoCb, this, 5.0, -6, -20, 20);
  _node->addDynamicDoubleParam(addMode("pitch_expo", _mode), &self::pitchExpoCb, this, 5.0, -6, -20, 20);
  _node->addDynamicDoubleParam(addMode("yaw_expo", _mode), &self::yawExpoCb, this, 5.0, -3, -20, 20);
  _node->addDynamicDoubleParam(addMode("throttle_expo", _mode), &self::throttleExpoCb, this, 5.0, 0, 0, 20);

  health_sub_ = _node->createSubscriber(topic::kVehicleHealth, &self::healthCb, this);
  multicopter_cmd_pub_ = _node->createPublisher<tobas_command_msgs::AngleThrottleVector>(topic::kAngleThrotVectorCmd);
  fixed_wing_cmd_pub_ = _node->createPublisher<tobas_command_msgs::msg::RollPitchSideSlipThrottleVector>(
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

  // Mo GPS flight
  if (!is_velocity_accurate_) {
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
    cmd->thrust_angle = std::min(thrust_angle, kModeChangeThrustAngle);
    multicopter_cmd_pub_->publish(std::move(cmd));
    return;
  }

  // Publish
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

void AngleSideSlipThrottleVectorController::maxRollCb(double _p)
{
  max_roll_ = st::deg2rad(_p);
}

void AngleSideSlipThrottleVectorController::maxRollRateCb(double _p)
{
  roll_filt_.setMaxVelocity(st::deg2rad(_p));
}

void AngleSideSlipThrottleVectorController::maxPitchCb(double _p)
{
  max_pitch_ = st::deg2rad(_p);
}

void AngleSideSlipThrottleVectorController::maxPitchRateCb(double _p)
{
  pitch_filt_.setMaxVelocity(st::deg2rad(_p));
}

void AngleSideSlipThrottleVectorController::maxSideSlipCb(double _p)
{
  max_side_slip_ = st::deg2rad(_p);
}

void AngleSideSlipThrottleVectorController::maxYawRateCb(double _p)
{
  max_yaw_rate_ = st::deg2rad(_p);
}

void AngleSideSlipThrottleVectorController::maxThrustAngleRateCb(double _p)
{
  thrust_angle_filt_.setMaxVelocity(st::deg2rad(_p));
}

void AngleSideSlipThrottleVectorController::rollExpoCb(double _p)
{
  roll_expo_ = _p / kExpoScale;
}

void AngleSideSlipThrottleVectorController::pitchExpoCb(double _p)
{
  pitch_expo_ = _p / kExpoScale;
}

void AngleSideSlipThrottleVectorController::yawExpoCb(double _p)
{
  yaw_expo_ = _p / kExpoScale;
}

void AngleSideSlipThrottleVectorController::throttleExpoCb(double _p)
{
  throt_expo_ = _p / kExpoScale;
}

void AngleSideSlipThrottleVectorController::healthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& _health)
{
  const auto passed = tobas_msgs::msg::VehicleHealth::PASSED;
  is_velocity_accurate_ =
    (_health->horizontal_position_accuracy == passed) && (_health->vertical_position_accuracy == passed);
}
}  // namespace rc
}  // namespace tobas

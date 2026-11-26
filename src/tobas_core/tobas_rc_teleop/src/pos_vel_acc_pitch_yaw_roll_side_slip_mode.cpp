// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rc_teleop/pos_vel_acc_pitch_yaw_roll_side_slip_mode.hpp"

#include <tobas_constants/ros.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas::rc
{
PosVelAccPitchYawRollSideSlipModeController::PosVelAccPitchYawRollSideSlipModeController()
{
}

bool PosVelAccPitchYawRollSideSlipModeController::requireHorizontalPosition() const
{
  return true;
}

bool PosVelAccPitchYawRollSideSlipModeController::requireVerticalPosition() const
{
  return true;
}

bool PosVelAccPitchYawRollSideSlipModeController::requireAttitude() const
{
  return true;
}

bool PosVelAccPitchYawRollSideSlipModeController::requireHeading() const
{
  return true;
}

void PosVelAccPitchYawRollSideSlipModeController::initialize(BaseNode* node, FlightMode mode)
{
  node->addDynamicDoubleParam(
    addMode("max_horizontal_velocity", mode), &self::maxHorizontalVelocityCb, this, 0.5, 12, 0, 20, " m/s");
  node->addDynamicDoubleParam(
    addMode("max_horizontal_jerk", mode), &self::maxHorizontalJerkCb, this, 5.0, 8, 1, 20, " m/s^3");
  node->addDynamicDoubleParam(
    addMode("max_vertical_velocity", mode), &self::maxVerticalVelocityCb, this, 0.5, 8, 0, 20, " m/s");
  node->addDynamicDoubleParam(
    addMode("max_vertical_jerk", mode), &self::maxVerticalJerkCb, this, 5.0, 8, 1, 20, " m/s^3");
  node->addDynamicDoubleParam(addMode("max_roll", mode), &self::maxRollCb, this, 5.0, 9, 1, 16, " deg");
  node->addDynamicDoubleParam(addMode("max_roll_rate", mode), &self::maxRollRateCb, this, 15.0, 6, 1, 12, " dps");
  node->addDynamicDoubleParam(addMode("max_pitch_rate", mode), &self::maxPitchRateCb, this, 15.0, 6, 1, 12, " dps");
  node->addDynamicDoubleParam(addMode("max_yaw_rate", mode), &self::maxYawRateCb, this, 15.0, 6, 1, 12, " dps");
  node->addDynamicDoubleParam(addMode("max_side_slip", mode), &self::maxSideSlipCb, this, 1.0, 6, 1, 12, " deg");
  node->addDynamicDoubleParam(
    addMode("max_position_error_down", mode), &self::maxPositionErrorDown, this, 0.5, 4, 0, 20, " m");
  node->addDynamicDoubleParam(
    addMode("horizontal_velocity_expo", mode), &self::horizontalVelocityExpoCb, this, 5.0, -6, -20, 20);
  node->addDynamicDoubleParam(
    addMode("vertical_velocity_expo", mode), &self::verticalVelocityExpoCb, this, 5.0, 0, -20, 20);
  node->addDynamicDoubleParam(addMode("roll_expo", mode), &self::rollExpoCb, this, 5.0, 0, -20, 20);
  node->addDynamicDoubleParam(addMode("yaw_expo", mode), &self::yawExpoCb, this, 5.0, -3, -20, 20);

  // publishers
  multicopter_cmd_pub_ = node->createPublisher<tobas_command_msgs::PosVelAccPitchYaw>(topic::kPosVelAccPitchYawCmd);
  fixed_wing_cmd_pub_ =
    node->createPublisher<tobas_command_msgs::msg::VelRollSideSlipMode>(topic::kVelRollSideSlipModeCmd);

  // subscribers
  multicopter_angle_cmd_sub_ = node->createSubscriber(topic::kAngleThrotVectorCmd, &self::multicopterAngleCmdCb, this);
  fixed_wing_manual_cmd_sub_ =
    node->createSubscriber(topic::kAileElevRudThrottleCmd, &self::fixedWingManualCmdCb, this);
  fixed_wing_angle_cmd_sub_ =
    node->createSubscriber(topic::kRollPitchSideSlipThrotVectorCmd, &self::fixedWingAngleCmdCb, this);
}

void PosVelAccPitchYawRollSideSlipModeController::reset(
  const builtin_interfaces::msg::Time& _stamp,
  const tobas_msgs::Odometry& _setpoint,
  bool _landed)
{
  t_last_rcin_ = _stamp;

  resetMulticopterControllers(_setpoint, _landed);
  resetFixedWingControllers(_setpoint, _landed);
}

void PosVelAccPitchYawRollSideSlipModeController::update(
  const tobas_msgs::RCInput& _rcin,
  const tobas_msgs::Odometry& _odom,
  bool _landed)
{
  // Update timestamp.
  const auto dt = (_rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = _rcin.header.stamp;

  // TODO : 逆遷移中にmanual modeからこのモードに入れたときに正しくモードを判断する
  // Mode transition
  switch (mode_) {
    case ControlMode::MULTICOPTER: {
      if (_rcin.sub_mode) {
        std::cout << "multicopter -> forward_flight" << std::endl;
        mode_ = ControlMode::FORWARD_FLIGHT;
        resetFixedWingControllers(_odom, _landed);
        break;
      }
      break;
    }
    case ControlMode::FORWARD_FLIGHT: {
      if (!_rcin.sub_mode) {
        std::cout << "forward_flight -> backward_transition" << std::endl;
        mode_ = ControlMode::BACKWARD_TRANSITION;
        break;
      }
      break;
    }
    case ControlMode::BACKWARD_TRANSITION: {
      if (
        _odom.twist.vel.x() <
        kBackwardTransitionFinishSpeed) {  // z方向速度をコマンドしているときにmulticopterにならないのを避けるためnormを取らない
        mode_ = ControlMode::MULTICOPTER;
        std::cout << "backward transition -> multicopter" << std::endl;
        resetMulticopterControllers(_odom, _landed);
        break;
      }
      if (_rcin.sub_mode) {
        std::cout << "backward_transition -> forward flight" << std::endl;
        mode_ = ControlMode::FORWARD_FLIGHT;
        resetFixedWingControllers(_odom, _landed);
        break;
      }
      break;
    }
    default:
      break;
  }

  // create & publish command
  switch (mode_) {
    case ControlMode::MULTICOPTER: {
      // Velocity x, y, z
      vx_filt_.update(expoRemapDead(_rcin.pitch, hor_vel_expo_, -max_hor_vel_, max_hor_vel_), dt);
      vy_filt_.update(-expoRemap(_rcin.roll, hor_vel_expo_, -max_hor_vel_, max_hor_vel_), dt);
      vz_filt_.update(expoRemapDead(_rcin.throttle, ver_vel_expo_, -max_ver_vel_, max_ver_vel_), dt);
      // Pitch
      pitch_filt_.setTargetPointAndUpdate(0.0, dt);
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

      // create command
      auto cmd = std::make_unique<tobas_command_msgs::PosVelAccPitchYaw>();
      cmd->header = _rcin.header;
      cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
      cmd->pos = tar_pos_W_;
      cmd->vel = tar_vel_W;
      cmd->acc = tar_acc_W;
      cmd->pitch = pitch_filt_.getTrajectoryPosition();
      cmd->yaw = tar_yaw_;

      // publish
      multicopter_cmd_pub_->publish(std::move(cmd));
      break;
    }
    case ControlMode::FORWARD_FLIGHT: {
      // Velocity x, z
      // 固定翼機のような操作感
      vx_filt_.update(expoRemapDead(_rcin.throttle, hor_vel_expo_, -max_hor_vel_, max_hor_vel_), dt);
      vz_filt_.update(expoRemapDead(-_rcin.pitch, ver_vel_expo_, -max_ver_vel_, max_ver_vel_), dt);

      // Roll
      roll_filt_.setTargetPointAndUpdate(expoRemapDead(_rcin.roll, roll_expo_, -max_roll_, max_roll_), dt);

      // create command
      auto cmd = std::make_unique<tobas_command_msgs::msg::VelRollSideSlipMode>();
      cmd->header = _rcin.header;
      cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
      cmd->vel_x = vx_filt_.getTrajectoryVelocity();
      cmd->vel_z = vz_filt_.getTrajectoryVelocity();
      cmd->roll = roll_filt_.getTrajectoryPosition();
      cmd->side_slip = expoRemapDead(_rcin.yaw, yaw_expo_, -max_side_slip_, max_side_slip_);
      cmd->mode = tobas_command_msgs::msg::VelRollSideSlipMode::FORWARD_FLIGHT;

      // publish
      fixed_wing_cmd_pub_->publish(std::move(cmd));
      break;
    }
    case ControlMode::BACKWARD_TRANSITION: {
      // multicopterのような操作感
      vx_filt_.update(0.0, dt);
      vz_filt_.update(expoRemapDead(_rcin.throttle, ver_vel_expo_, -max_ver_vel_, max_ver_vel_), dt);

      // Roll
      roll_filt_.setTargetPointAndUpdate(expoRemapDead(_rcin.roll, roll_expo_, -max_roll_, max_roll_), dt);

      // create command
      auto cmd = std::make_unique<tobas_command_msgs::msg::VelRollSideSlipMode>();
      cmd->header = _rcin.header;
      cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
      cmd->vel_x = 0.0;  // not used
      cmd->vel_z = vz_filt_.getTrajectoryVelocity();
      cmd->roll = roll_filt_.getTrajectoryPosition();
      cmd->side_slip = expoRemapDead(_rcin.yaw, yaw_expo_, -max_side_slip_, max_side_slip_);
      cmd->mode = tobas_command_msgs::msg::VelRollSideSlipMode::BACKWARD_TRANSITION;

      // publish
      fixed_wing_cmd_pub_->publish(std::move(cmd));
      break;
    }
    default:
      break;
  }
}

void PosVelAccPitchYawRollSideSlipModeController::multicopterAngleCmdCb(
  const tobas_command_msgs::AngleThrottleVector::ConstSharedPtr&)
{
  mode_ = ControlMode::MULTICOPTER;
}

void PosVelAccPitchYawRollSideSlipModeController::fixedWingManualCmdCb(
  const tobas_command_msgs::msg::AileElevRudThrottle::ConstSharedPtr&)
{
  mode_ = ControlMode::FORWARD_FLIGHT;
}

void PosVelAccPitchYawRollSideSlipModeController::fixedWingAngleCmdCb(
  const tobas_command_msgs::msg::RollPitchSideSlipThrottleVector::ConstSharedPtr&)
{
  mode_ = ControlMode::FORWARD_FLIGHT;
}

void PosVelAccPitchYawRollSideSlipModeController::resetFixedWingControllers(const tobas_msgs::Odometry& _setpoint, bool)
{
  const auto [roll, pitch, yaw] = _setpoint.frame.M.getRPY();
  roll_filt_.resetCurrentTrajectoryPoint(roll);
}

void PosVelAccPitchYawRollSideSlipModeController::resetMulticopterControllers(
  const tobas_msgs::Odometry& _setpoint,
  bool _landed)
{
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

void PosVelAccPitchYawRollSideSlipModeController::maxHorizontalVelocityCb(double _p)
{
  max_hor_vel_ = _p;
  vx_filt_.setMaxVelocity(_p);
  vy_filt_.setMaxVelocity(_p);
}

void PosVelAccPitchYawRollSideSlipModeController::maxHorizontalJerkCb(double _p)
{
  vx_filt_.setMaxJerk(_p);
  vy_filt_.setMaxJerk(_p);
}

void PosVelAccPitchYawRollSideSlipModeController::maxVerticalVelocityCb(const double _p)
{
  max_ver_vel_ = _p;
  vz_filt_.setMaxVelocity(_p);
}

void PosVelAccPitchYawRollSideSlipModeController::maxVerticalJerkCb(const double _p)
{
  vz_filt_.setMaxJerk(_p);
}

void PosVelAccPitchYawRollSideSlipModeController::maxRollCb(const double _p)
{
  max_roll_ = st::deg2rad(_p);
}

void PosVelAccPitchYawRollSideSlipModeController::maxRollRateCb(const double _p)
{
  roll_filt_.setMaxVelocity(st::deg2rad(_p));
}

void PosVelAccPitchYawRollSideSlipModeController::maxPitchRateCb(const double _p)
{
  pitch_filt_.setMaxVelocity(st::deg2rad(_p));
}

void PosVelAccPitchYawRollSideSlipModeController::maxYawRateCb(const double _p)
{
  max_yaw_rate_ = st::deg2rad(_p);
}

void PosVelAccPitchYawRollSideSlipModeController::maxSideSlipCb(const double _p)
{
  max_side_slip_ = st::deg2rad(_p);
}

void PosVelAccPitchYawRollSideSlipModeController::maxPositionErrorDown(const double _p)
{
  max_ep_down_ = _p;
}

void PosVelAccPitchYawRollSideSlipModeController::horizontalVelocityExpoCb(const double _p)
{
  hor_vel_expo_ = _p / kExpoScale;
}

void PosVelAccPitchYawRollSideSlipModeController::verticalVelocityExpoCb(const double _p)
{
  ver_vel_expo_ = _p / kExpoScale;
}

void PosVelAccPitchYawRollSideSlipModeController::rollExpoCb(const double _p)
{
  roll_expo_ = _p / kExpoScale;
}

void PosVelAccPitchYawRollSideSlipModeController::yawExpoCb(const double _p)
{
  yaw_expo_ = _p / kExpoScale;
}
}  // namespace tobas::rc

#include "tobas_rc_teleop/pos_vel_acc_pitch_yaw.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas_rc_teleop
{
PosVelAccPitchYawController::PosVelAccPitchYawController()
{
}

bool PosVelAccPitchYawController::requirePosition()
{
  return true;
}

bool PosVelAccPitchYawController::requireVelocity()
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

void PosVelAccPitchYawController::initialize(tobas::BaseNode* node, tobas::FlightMode mode)
{
  node->addDynamicDoubleParam(
    addMode("max_horizontal_velocity", mode), &self::maxHorizontalVelocityCb, this, 0.5, 12, 0, 20, " m/s");
  node->addDynamicDoubleParam(
    addMode("max_horizontal_accel", mode), &self::maxHorizontalAccelCb, this, 1., 10, 1, 20, " m/s^2");
  node->addDynamicDoubleParam(
    addMode("max_vertical_velocity", mode), &self::maxVerticalVelocityCb, this, 0.5, 8, 0, 20, " m/s");
  node->addDynamicDoubleParam(addMode("max_pitch", mode), &self::maxPitchCb, this, 10., 9, 1, 18, " deg");
  node->addDynamicDoubleParam(addMode("max_pitch_rate", mode), &self::maxPitchRateCb, this, 20., 9, 1, 18, " dps");
  node->addDynamicDoubleParam(addMode("max_yaw_rate", mode), &self::maxYawRateCb, this, 20., 9, 1, 18, " dps");
  node->addDynamicDoubleParam(
    addMode("max_position_error_down", mode), &self::maxPositionErrorDown, this, 0.5, 4, 0, 20, " m");
  node->addDynamicDoubleParam(
    addMode("horizontal_velocity_expo", mode), &self::horizontalVelocityExpoCb, this, 5., -6, -20, 20);
  node->addDynamicDoubleParam(
    addMode("vertical_velocity_expo", mode), &self::verticalVelocityExpoCb, this, 5., 0, -20, 20);
  node->addDynamicDoubleParam(addMode("pitch_expo", mode), &self::pitchExpoCb, this, 5., 0, -20, 20);
  node->addDynamicDoubleParam(addMode("yaw_expo", mode), &self::yawExpoCb, this, 5., -3, -20, 20);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::PosVelAccPitchYaw>(tobas::topic::kPosVelAccPitchYawCmd);
}

void PosVelAccPitchYawController::reset(
  const builtin_interfaces::msg::Time& stamp,
  const tobas_msgs::Odometry& odom,
  bool landed)
{
  t_last_rcin_ = stamp;

  const auto [roll, pitch, yaw] = odom.frame.M.getRPY();

  const auto R_G_B = kdl::Rotation::RPY(roll, pitch, 0.);
  const auto cur_vel_G = R_G_B * odom.twist.vel;
  vx_filt_.resetCurrentTrajectoryPoint(cur_vel_G.x());
  vy_filt_.resetCurrentTrajectoryPoint(cur_vel_G.y());

  tar_pos_W_ = odom.frame.p;
  if (landed) {
    tar_pos_W_.z() -= max_ep_down_;
  }

  pitch_filt_.resetCurrentTrajectoryPoint(pitch);
  tar_yaw_ = odom.frame.M.getYaw();
}

void PosVelAccPitchYawController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed)
{
  // Update timestamp
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // Velocity-X & Pitch
  if (rcin.sub_mode)  // Translation mode
  {
    vx_filt_.setTargetPosition(expoRemapDead(rcin.pitch, hor_vel_expo_, -max_hor_vel_, max_hor_vel_));
    pitch_filt_.setTargetPosition(0.);
  }
  else  // Rotation mode
  {
    vx_filt_.setTargetPosition(0.);
    pitch_filt_.setTargetPosition(expoRemapDead(rcin.pitch, pitch_expo_, -max_pitch_, max_pitch_));
  }
  vx_filt_.update(dt);
  pitch_filt_.update(dt);

  // Velocity-Y
  vy_filt_.setTargetPosition(-expoRemap(rcin.roll, hor_vel_expo_, -max_hor_vel_, max_hor_vel_));
  vy_filt_.update(dt);

  // Velocity-Z
  const auto vz = expoRemap(rcin.throttle, ver_vel_expo_, -max_ver_vel_, max_ver_vel_);

  // Yaw
  const auto yawrate = expoRemapDead(rcin.yaw, yaw_expo_, -max_yaw_rate_, max_yaw_rate_);
  tar_yaw_ += yawrate * dt;

  // Compute the velocity wrt. the world frame
  const kdl::Vector tar_vel_G(vx_filt_.getTrajectoryPosition(), vy_filt_.getTrajectoryPosition(), vz);
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

  // Create a command
  auto cmd = std::make_unique<tobas_command_msgs::PosVelAccPitchYaw>();
  cmd->header = rcin.header;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
  cmd->pos = tar_pos_W_;
  cmd->vel = tar_vel_W;
  cmd->acc.setZero();
  cmd->pitch = pitch_filt_.getTrajectoryPosition();
  cmd->yaw = tar_yaw_;

  // Publish the command
  cmd_pub_->publish(std::move(cmd));
}

bool PosVelAccPitchYawController::maxHorizontalVelocityCb(const double& p)
{
  max_hor_vel_ = p;
  return true;
}

bool PosVelAccPitchYawController::maxHorizontalAccelCb(const double& p)
{
  vx_filt_.setMaxVelocity(p);
  vy_filt_.setMaxVelocity(p);
  return true;
}

bool PosVelAccPitchYawController::maxVerticalVelocityCb(const double& p)
{
  max_ver_vel_ = p;
  return true;
}

bool PosVelAccPitchYawController::maxPitchCb(const double& p)
{
  max_pitch_ = tbs::deg2rad(p);
  return true;
}

bool PosVelAccPitchYawController::maxPitchRateCb(const double& p)
{
  pitch_filt_.setMaxVelocity(tbs::deg2rad(p));
  return true;
}

bool PosVelAccPitchYawController::maxYawRateCb(const double& p)
{
  max_yaw_rate_ = tbs::deg2rad(p);
  return true;
}

bool PosVelAccPitchYawController::maxPositionErrorDown(const double& p)
{
  max_ep_down_ = p;
  return true;
}

bool PosVelAccPitchYawController::horizontalVelocityExpoCb(const double& p)
{
  hor_vel_expo_ = p / kExpoScale;
  return true;
}

bool PosVelAccPitchYawController::verticalVelocityExpoCb(const double& p)
{
  ver_vel_expo_ = p / kExpoScale;
  return true;
}

bool PosVelAccPitchYawController::pitchExpoCb(const double& p)
{
  pitch_expo_ = p / kExpoScale;
  return true;
}

bool PosVelAccPitchYawController::yawExpoCb(const double& p)
{
  yaw_expo_ = p / kExpoScale;
  return true;
}
}  // namespace tobas_rc_teleop

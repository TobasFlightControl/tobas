#include "tobas_rc_teleop/pos_vel_acc_yaw.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas_rc_teleop
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

void PosVelAccYawController::initialize(tobas::BaseNode* node, tobas::FlightMode mode)
{
  node->addDynamicDoubleParam(
    addMode("max_horizontal_velocity", mode), &self::maxHorizontalVelocityCb, this, 0.5, 12, 0, 20, " m/s");
  node->addDynamicDoubleParam(
    addMode("max_horizontal_accel", mode), &self::maxHorizontalAccelCb, this, 1., 10, 1, 20, " m/s^2");
  node->addDynamicDoubleParam(
    addMode("max_vertical_velocity", mode), &self::maxVerticalVelocityCb, this, 0.5, 8, 0, 20, " m/s");
  node->addDynamicDoubleParam(
    addMode("max_vertical_accel", mode), &self::maxVerticalAccelCb, this, 1., 10, 1, 20, " m/s^2");
  node->addDynamicDoubleParam(addMode("max_heading_rate", mode), &self::maxHeadingRateCb, this, 20., 9, 1, 18, " dps");
  node->addDynamicDoubleParam(
    addMode("max_position_error_down", mode), &self::maxPositionErrorDown, this, 0.5, 4, 0, 20, " m");
  node->addDynamicDoubleParam(
    addMode("horizontal_velocity_expo", mode), &self::horizontalVelocityExpoCb, this, 5., -6, -20, 20);
  node->addDynamicDoubleParam(
    addMode("vertical_velocity_expo", mode), &self::verticalVelocityExpoCb, this, 5., 0, -20, 20);
  node->addDynamicDoubleParam(addMode("heading_expo", mode), &self::headingExpoCb, this, 5., -3, -20, 20);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::PosVelAccYaw>(tobas::topic::kPosVelAccYawCmd);
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
  vx_filt_.resetCurrentTrajectoryPoint(tar_vel_G.x());
  vy_filt_.resetCurrentTrajectoryPoint(tar_vel_G.y());
  vz_filt_.resetCurrentTrajectoryPoint(tar_vel_G.z());

  tar_pos_W_ = setpoint.frame.p;

  if (landed) {
    vz_filt_.resetCurrentTrajectoryPoint(-max_ver_vel_);
    tar_pos_W_.z() -= max_ep_down_;
  }

  tar_yaw_ = yaw;
}

void PosVelAccYawController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed)
{
  // 時刻を更新
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // Velocity
  vx_filt_.setTargetPointAndUpdate(expoRemapDead(rcin.pitch, hor_vel_expo_, -max_hor_vel_, max_hor_vel_), dt);
  vy_filt_.setTargetPointAndUpdate(-expoRemapDead(rcin.roll, hor_vel_expo_, -max_hor_vel_, max_hor_vel_), dt);
  vz_filt_.setTargetPointAndUpdate(expoRemapDead(rcin.throttle, ver_vel_expo_, -max_ver_vel_, max_ver_vel_), dt);

  // Yaw
  const auto yawrate = expoRemapDead(rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_);
  tar_yaw_ += yawrate * dt;

  // Compute the velocity and acceleration wrt. the world frame
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

  // Create a command
  auto cmd = std::make_unique<tobas_command_msgs::PosVelAccYaw>();
  cmd->header = rcin.header;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
  cmd->pos = tar_pos_W_;
  cmd->vel = tar_vel_W;
  cmd->acc.setZero();
  cmd->yaw = tar_yaw_;

  // Publish the command
  cmd_pub_->publish(std::move(cmd));
}

bool PosVelAccYawController::maxHorizontalVelocityCb(const double& p)
{
  max_hor_vel_ = p;
  return true;
}

bool PosVelAccYawController::maxHorizontalAccelCb(const double& p)
{
  vx_filt_.setMaxVelocity(p);
  vy_filt_.setMaxVelocity(p);
  return true;
}

bool PosVelAccYawController::maxVerticalVelocityCb(const double& p)
{
  max_ver_vel_ = p;
  return true;
}

bool PosVelAccYawController::maxVerticalAccelCb(const double& p)
{
  vz_filt_.setMaxVelocity(p);
  return true;
}

bool PosVelAccYawController::maxHeadingRateCb(const double& p)
{
  max_head_rate_ = tbs::deg2rad(p);
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
}  // namespace tobas_rc_teleop

#include "tobas_rc_teleop/accel_pitch_yaw.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas_rc_teleop
{
AccelPitchYawController::AccelPitchYawController()
{
}

bool AccelPitchYawController::requirePosition()
{
  return false;
}

bool AccelPitchYawController::requireVelocity()
{
  return false;
}

bool AccelPitchYawController::requireAttitude()
{
  return true;
}

bool AccelPitchYawController::requireHeading()
{
  return true;
}

void AccelPitchYawController::initialize(tobas::BaseNode* node, tobas::FlightMode mode)
{
  node->addDynamicDoubleParam(
    addMode("max_horizontal_accel", mode), &self::maxHorizontalAccelCb, this, 0.5, 10, 1, 20, " m/s^2");
  node->addDynamicDoubleParam(
    addMode("max_horizontal_jerk", mode), &self::maxHorizontalJerkCb, this, 5., 8, 1, 20, " m/s^3");
  node->addDynamicDoubleParam(
    addMode("max_vertical_accel", mode), &self::maxVerticalAccelCb, this, 0.5, 16, 1, 20, " m/s^2");
  node->addDynamicIntParam(addMode("max_pitch", mode), &self::maxPitchCb, this, 90, 0, 180, " deg");
  node->addDynamicIntParam(addMode("max_pitch_rate", mode), &self::maxPitchRateCb, this, 180, 0, 360, " dps");
  node->addDynamicIntParam(addMode("max_yaw_rate", mode), &self::maxYawRateCb, this, 180, 0, 360, " dps");
  node->addDynamicIntParam(
    addMode("horizontal_accel_expo", mode), &self::horizontalAccelExpoCb, this, -30, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(
    addMode("vertical_accel_expo", mode), &self::verticalAccelExpoCb, this, 0, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("yaw_expo", mode), &self::yawExpoCb, this, -15, -kExpoScale, kExpoScale);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::AccelPitchYaw>(tobas::topic::kAccelPitchYawCmd);
}

void AccelPitchYawController::reset(const tobas_msgs::Odometry& odom)
{
  t_last_rcin_ = odom.header.stamp;

  ax_filt_.resetCurrentTrajectoryPoint(0.);
  ay_filt_.resetCurrentTrajectoryPoint(0.);

  pitch_filt_.resetCurrentTrajectoryPoint(odom.frame.M.getPitch());
  tar_yaw_ = odom.frame.M.getYaw();
}

void AccelPitchYawController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry&)
{
  // Update timestamp
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // Accel-X & Pitch
  if (rcin.sub_mode)  // Translation mode
  {
    ax_filt_.setTargetPosition(expoRemap(rcin.pitch, hor_acc_expo_, -max_hor_acc_, max_hor_acc_));
    pitch_filt_.setTargetPosition(0.);
  }
  else  // Rotation mode
  {
    ax_filt_.setTargetPosition(0.);
    pitch_filt_.setTargetPosition(remapDead(rcin.pitch, -max_pitch_, max_pitch_));
  }
  ax_filt_.update(dt);
  pitch_filt_.update(dt);

  // Accel-Y
  ay_filt_.setTargetPosition(-expoRemap(rcin.roll, hor_acc_expo_, -max_hor_acc_, max_hor_acc_));
  ay_filt_.update(dt);

  // Accel-Z
  const auto az = expoRemap(rcin.throttle, ver_acc_expo_, -max_ver_acc_, max_ver_acc_);

  // Yaw
  const auto yawrate = expoRemapDead(rcin.yaw, yaw_expo_, -max_yaw_rate_, max_yaw_rate_);
  tar_yaw_ += yawrate * dt;

  // Compute the acceleration wrt. the world frame
  const kdl::Vector tar_acc_G(ax_filt_.getTrajectoryPosition(), ay_filt_.getTrajectoryPosition(), az);
  const auto tar_acc_W = kdl::Rotation::RotZ(tar_yaw_) * tar_acc_G;

  // Create a command
  auto cmd = std::make_unique<tobas_command_msgs::AccelPitchYaw>();
  cmd->header = rcin.header;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
  cmd->accel = tar_acc_W;
  cmd->pitch = pitch_filt_.getTrajectoryPosition();
  cmd->yaw = tar_yaw_;

  // Publish the command
  cmd_pub_->publish(std::move(cmd));
}

bool AccelPitchYawController::maxHorizontalAccelCb(const double& p)
{
  max_hor_acc_ = p;
  return true;
}

bool AccelPitchYawController::maxHorizontalJerkCb(const double& p)
{
  ax_filt_.setMaxVelocity(p);
  ay_filt_.setMaxVelocity(p);
  return true;
}

bool AccelPitchYawController::maxVerticalAccelCb(const double& p)
{
  max_ver_acc_ = p;
  return true;
}

bool AccelPitchYawController::maxPitchCb(const long& p)
{
  max_pitch_ = tbs::deg2rad(p);
  return true;
}

bool AccelPitchYawController::maxPitchRateCb(const long& p)
{
  pitch_filt_.setMaxVelocity(tbs::deg2rad(p));
  return true;
}

bool AccelPitchYawController::maxYawRateCb(const long& p)
{
  max_yaw_rate_ = tbs::deg2rad(p);
  return true;
}

bool AccelPitchYawController::horizontalAccelExpoCb(const long& p)
{
  hor_acc_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool AccelPitchYawController::verticalAccelExpoCb(const long& p)
{
  ver_acc_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool AccelPitchYawController::yawExpoCb(const long& p)
{
  yaw_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}
}  // namespace tobas_rc_teleop

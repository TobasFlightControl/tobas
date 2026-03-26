#include "tobas_rc_teleop/accel_pitch_yaw.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace rc
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

void AccelPitchYawController::initialize(BaseNode* node, FlightMode mode)
{
  node->addDynamicDoubleParam(
    addMode("max_horizontal_accel", mode), &self::maxHorizontalAccelCb, this, 0.5, 10, 1, 20, " m/s^2");
  node->addDynamicDoubleParam(
    addMode("max_horizontal_jerk", mode), &self::maxHorizontalJerkCb, this, 5., 8, 1, 20, " m/s^3");
  node->addDynamicDoubleParam(
    addMode("max_vertical_accel", mode), &self::maxVerticalAccelCb, this, 0.5, 16, 1, 20, " m/s^2");
  node->addDynamicDoubleParam(addMode("max_pitch", mode), &self::maxPitchCb, this, 10., 9, 1, 18, " deg");
  node->addDynamicDoubleParam(addMode("max_pitch_rate", mode), &self::maxPitchRateCb, this, 20., 9, 1, 18, " dps");
  node->addDynamicDoubleParam(addMode("max_yaw_rate", mode), &self::maxYawRateCb, this, 20., 9, 1, 18, " dps");
  node->addDynamicDoubleParam(
    addMode("horizontal_accel_expo", mode), &self::horizontalAccelExpoCb, this, 5., -6, -20, 20);
  node->addDynamicDoubleParam(addMode("vertical_accel_expo", mode), &self::verticalAccelExpoCb, this, 5., 0, -20, 20);
  node->addDynamicDoubleParam(addMode("pitch_expo", mode), &self::pitchExpoCb, this, 5., 0, -20, 20);
  node->addDynamicDoubleParam(addMode("yaw_expo", mode), &self::yawExpoCb, this, 5., -3, -20, 20);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::AccelPitchYaw>(topic::kAccelPitchYawCmd);
}

void AccelPitchYawController::reset(const builtin_interfaces::msg::Time& stamp, const tobas_msgs::Odometry& setpoint, bool)
{
  t_last_rcin_ = stamp;

  ax_filt_.resetCurrentTrajectoryPoint(0.);
  ay_filt_.resetCurrentTrajectoryPoint(0.);

  pitch_filt_.resetCurrentTrajectoryPoint(setpoint.frame.M.getPitch());
  tar_yaw_ = setpoint.frame.M.getYaw();
}

void AccelPitchYawController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry&, bool)
{
  // Update timestamp
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // Accel-X & Pitch
  if (rcin.sub_mode)  // Translation mode
  {
    ax_filt_.setTargetPointAndUpdate(expoRemap(rcin.pitch, hor_acc_expo_, -max_hor_acc_, max_hor_acc_), dt);
    pitch_filt_.setTargetPointAndUpdate(0., dt);
  }
  else  // Rotation mode
  {
    ax_filt_.setTargetPointAndUpdate(0., dt);
    pitch_filt_.setTargetPointAndUpdate(expoRemapDead(rcin.pitch, pitch_expo_, -max_pitch_, max_pitch_), dt);
  }

  // Accel-Y
  ay_filt_.setTargetPointAndUpdate(-expoRemap(rcin.roll, hor_acc_expo_, -max_hor_acc_, max_hor_acc_), dt);

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

bool AccelPitchYawController::maxPitchCb(const double& p)
{
  max_pitch_ = st::deg2rad(p);
  return true;
}

bool AccelPitchYawController::maxPitchRateCb(const double& p)
{
  pitch_filt_.setMaxVelocity(st::deg2rad(p));
  return true;
}

bool AccelPitchYawController::maxYawRateCb(const double& p)
{
  max_yaw_rate_ = st::deg2rad(p);
  return true;
}

bool AccelPitchYawController::horizontalAccelExpoCb(const double& p)
{
  hor_acc_expo_ = p / kExpoScale;
  return true;
}

bool AccelPitchYawController::verticalAccelExpoCb(const double& p)
{
  ver_acc_expo_ = p / kExpoScale;
  return true;
}

bool AccelPitchYawController::pitchExpoCb(const double& p)
{
  pitch_expo_ = p / kExpoScale;
  return true;
}

bool AccelPitchYawController::yawExpoCb(const double& p)
{
  yaw_expo_ = p / kExpoScale;
  return true;
}
}  // namespace rc
}  // namespace tobas

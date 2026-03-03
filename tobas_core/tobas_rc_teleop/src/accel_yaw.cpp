#include "tobas_rc_teleop/accel_yaw.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas_rc_teleop
{
AccelYawController::AccelYawController()
{
}

bool AccelYawController::requirePosition()
{
  return false;
}

bool AccelYawController::requireVelocity()
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

void AccelYawController::initialize(tobas::BaseNode* node, tobas::FlightMode mode)
{
  node->addDynamicDoubleParam(
    addMode("max_horizontal_accel", mode), &self::maxHorizontalAccelCb, this, 0.5, 10, 1, 20, " m/s^2");
  node->addDynamicDoubleParam(
    addMode("max_horizontal_jerk", mode), &self::maxHorizontalJerkCb, this, 5., 8, 1, 20, " m/s^3");
  node->addDynamicDoubleParam(
    addMode("max_vertical_accel", mode), &self::maxVerticalAccelCb, this, 0.5, 16, 1, 20, " m/s^2");
  node->addDynamicIntParam(addMode("max_heading_rate", mode), &self::maxHeadingRateCb, this, 180, 0, 360, " dps");
  node->addDynamicIntParam(
    addMode("horizontal_accel_expo", mode), &self::horizontalAccelExpoCb, this, -30, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(
    addMode("vertical_accel_expo", mode), &self::verticalAccelExpoCb, this, 0, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("heading_expo", mode), &self::headingExpoCb, this, -15, -kExpoScale, kExpoScale);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::AccelYaw>(tobas::topic::kAccelYawCmd);
}

void AccelYawController::reset(const tobas_msgs::Odometry& odom)
{
  t_last_rcin_ = odom.header.stamp;

  ax_filt_.resetCurrentTrajectoryPoint(0.);
  ay_filt_.resetCurrentTrajectoryPoint(0.);

  tar_yaw_ = odom.frame.M.getYaw();
}

void AccelYawController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry&)
{
  // Update timestamp
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // Horizontal acceleration
  ax_filt_.setTargetPosition(expoRemap(rcin.pitch, hor_acc_expo_, -max_hor_acc_, max_hor_acc_));
  ay_filt_.setTargetPosition(-expoRemap(rcin.roll, hor_acc_expo_, -max_hor_acc_, max_hor_acc_));

  // Vertical acceleration
  const auto az = expoRemap(rcin.throttle, ver_acc_expo_, -max_ver_acc_, max_ver_acc_);

  // Yaw
  const auto yawrate = expoRemapDead(rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_);
  tar_yaw_ += yawrate * dt;

  // Compute the acceleration wrt. the world frame
  const kdl::Vector tar_acc_G(ax_filt_.getTrajectoryPosition(), ay_filt_.getTrajectoryPosition(), az);
  const auto tar_acc_W = kdl::Rotation::RotZ(tar_yaw_) * tar_acc_G;

  // Create a command
  auto cmd = std::make_unique<tobas_command_msgs::AccelYaw>();
  cmd->header = rcin.header;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
  cmd->accel = tar_acc_W;
  cmd->yaw = tar_yaw_;

  // Publish the command
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

bool AccelYawController::maxHeadingRateCb(const long& p)
{
  max_head_rate_ = tbs::deg2rad(p);
  return true;
}

bool AccelYawController::horizontalAccelExpoCb(const long& p)
{
  hor_acc_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool AccelYawController::verticalAccelExpoCb(const long& p)
{
  ver_acc_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool AccelYawController::headingExpoCb(const long& p)
{
  head_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}
}  // namespace tobas_rc_teleop

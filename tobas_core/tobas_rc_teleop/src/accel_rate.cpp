#include "tobas_rc_teleop/accel_rate.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas_rc_teleop
{
AccelRateController::AccelRateController()
{
}

bool AccelRateController::requirePosition()
{
  return false;
}

bool AccelRateController::requireVelocity()
{
  return false;
}

bool AccelRateController::requireAttitude()
{
  return true;
}

bool AccelRateController::requireHeading()
{
  return false;
}

void AccelRateController::initialize(tobas::BaseNode* node, tobas::FlightMode mode)
{
  node->addDynamicDoubleParam(
    addMode("max_horizontal_accel", mode), &self::maxHorizontalAccelCb, this, 0.5, 10, 1, 20, " m/s^2");
  node->addDynamicDoubleParam(
    addMode("max_vertical_accel", mode), &self::maxVerticalAccelCb, this, 0.5, 16, 1, 20, " m/s^2");
  node->addDynamicDoubleParam(addMode("max_attitude_rate", mode), &self::maxAttitudeRateCb, this, 20., 9, 1, 18, " dps");
  node->addDynamicDoubleParam(addMode("max_heading_rate", mode), &self::maxHeadingRateCb, this, 20., 9, 1, 18, " dps");
  node->addDynamicDoubleParam(
    addMode("horizontal_accel_expo", mode), &self::horizontalAccelExpoCb, this, 5., -6, -20, 20);
  node->addDynamicDoubleParam(addMode("vertical_accel_expo", mode), &self::verticalAccelExpoCb, this, 5., 0, -20, 20);
  node->addDynamicDoubleParam(addMode("attitude_expo", mode), &self::attitudeExpoCb, this, 5., 0, -20, 20);
  node->addDynamicDoubleParam(addMode("heading_expo", mode), &self::headingExpoCb, this, 5., -3, -20, 20);

  accel_pub_ = node->createPublisher<tobas_command_msgs::Accel>(tobas::topic::kAccelCmd);
  rate_pub_ = node->createPublisher<tobas_command_msgs::Rate>(tobas::topic::kRateCmd);
}

void AccelRateController::reset(const tobas_msgs::Odometry&, bool)
{
}

void AccelRateController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool)
{
  // Horizontal acceleration & Attitude rate
  if (rcin.sub_mode)  // Translation mode
  {
    tar_acc_G_.x(expoRemap(rcin.pitch, hor_acc_expo_, -max_hor_acc_, max_hor_acc_));
    tar_acc_G_.y(-expoRemap(rcin.roll, hor_acc_expo_, -max_hor_acc_, max_hor_acc_));
    tar_gyro_B_.x(0.);
    tar_gyro_B_.y(0.);
  }
  else  // Rotation mode
  {
    tar_gyro_B_.x(expoRemap(rcin.roll, atti_expo_, -max_atti_rate_, max_atti_rate_));
    tar_gyro_B_.y(expoRemap(rcin.pitch, atti_expo_, -max_atti_rate_, max_atti_rate_));
    tar_acc_G_.x(0.);
    tar_acc_G_.y(0.);
  }

  // Vertical acceleration
  tar_acc_G_.z(expoRemap(rcin.throttle, ver_acc_expo_, -max_ver_acc_, max_ver_acc_));

  // Heading rate
  tar_gyro_B_.z(expoRemap(rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_));

  // Compute the acceleration wrt. the world frame
  const auto cur_yaw = odom.frame.M.getYaw();
  const auto tar_acc_W = kdl::Rotation::RotZ(cur_yaw) * tar_acc_G_;

  // Publish commands
  publishAccel(rcin.header.stamp, tar_acc_W);
  publishRate(rcin.header.stamp, tar_gyro_B_);
}

void AccelRateController::publishAccel(const builtin_interfaces::msg::Time& stamp, const kdl::Vector& acc)
{
  auto cmd = std::make_unique<tobas_command_msgs::Accel>();
  cmd->header.stamp = stamp;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
  cmd->accel = acc;

  accel_pub_->publish(std::move(cmd));
}

void AccelRateController::publishRate(const builtin_interfaces::msg::Time& stamp, const kdl::Vector& rate)
{
  auto cmd = std::make_unique<tobas_command_msgs::Rate>();
  cmd->header.stamp = stamp;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
  cmd->rate = rate;

  rate_pub_->publish(std::move(cmd));
}

bool AccelRateController::maxHorizontalAccelCb(const double& p)
{
  max_hor_acc_ = p;
  return true;
}

bool AccelRateController::maxVerticalAccelCb(const double& p)
{
  max_ver_acc_ = p;
  return true;
}

bool AccelRateController::maxAttitudeRateCb(const double& p)
{
  max_atti_rate_ = tbs::deg2rad(p);
  return true;
}

bool AccelRateController::maxHeadingRateCb(const double& p)
{
  max_head_rate_ = tbs::deg2rad(p);
  return true;
}

bool AccelRateController::horizontalAccelExpoCb(const double& p)
{
  hor_acc_expo_ = p / kExpoScale;
  return true;
}

bool AccelRateController::verticalAccelExpoCb(const double& p)
{
  ver_acc_expo_ = p / kExpoScale;
  return true;
}

bool AccelRateController::attitudeExpoCb(const double& p)
{
  atti_expo_ = p / kExpoScale;
  return true;
}

bool AccelRateController::headingExpoCb(const double& p)
{
  head_expo_ = p / kExpoScale;
  return true;
}
}  // namespace tobas_rc_teleop

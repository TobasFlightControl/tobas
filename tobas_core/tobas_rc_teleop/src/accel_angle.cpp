#include "tobas_rc_teleop/accel_angle.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace rc
{
AccelAngleController::AccelAngleController()
{
}

bool AccelAngleController::requirePosition()
{
  return false;
}

bool AccelAngleController::requireVelocity()
{
  return false;
}

bool AccelAngleController::requireAttitude()
{
  return true;
}

bool AccelAngleController::requireHeading()
{
  return true;
}

void AccelAngleController::initialize(tobas::BaseNode* node, tobas::FlightMode mode)
{
  node->addDynamicDoubleParam(
    addMode("max_horizontal_accel", mode), &self::maxHorizontalAccelCb, this, 0.5, 10, 1, 20, " m/s^2");
  node->addDynamicDoubleParam(
    addMode("max_horizontal_jerk", mode), &self::maxHorizontalJerkCb, this, 5., 8, 1, 20, " m/s^3");
  node->addDynamicDoubleParam(
    addMode("max_vertical_accel", mode), &self::maxVerticalAccelCb, this, 0.5, 16, 1, 20, " m/s^2");
  node->addDynamicDoubleParam(addMode("max_attitude", mode), &self::maxAttitudeCb, this, 10., 9, 1, 18, " deg");
  node->addDynamicDoubleParam(addMode("max_attitude_rate", mode), &self::maxAttitudeRateCb, this, 20., 9, 1, 18, " dps");
  node->addDynamicDoubleParam(addMode("max_heading_rate", mode), &self::maxHeadingRateCb, this, 20., 9, 1, 18, " dps");
  node->addDynamicDoubleParam(
    addMode("horizontal_accel_expo", mode), &self::horizontalAccelExpoCb, this, 5., -6, -20, 20);
  node->addDynamicDoubleParam(addMode("vertical_accel_expo", mode), &self::verticalAccelExpoCb, this, 5., 0, -20, 20);
  node->addDynamicDoubleParam(addMode("attitude_expo", mode), &self::attitudeExpoCb, this, 5., 0, -20, 20);
  node->addDynamicDoubleParam(addMode("heading_expo", mode), &self::headingExpoCb, this, 5., -3, -20, 20);

  accel_pub_ = node->createPublisher<tobas_command_msgs::Accel>(tobas::topic::kAccelCmd);
  angle_pub_ = node->createPublisher<tobas_command_msgs::Angle>(tobas::topic::kAngleCmd);
}

void AccelAngleController::reset(const builtin_interfaces::msg::Time& stamp, const tobas_msgs::Odometry& setpoint, bool)
{
  t_last_rcin_ = stamp;

  ax_filt_.resetCurrentTrajectoryPoint(0.);
  ay_filt_.resetCurrentTrajectoryPoint(0.);

  const auto [roll, pitch, yaw] = setpoint.frame.M.getRPY();
  roll_filt_.resetCurrentTrajectoryPoint(roll);
  pitch_filt_.resetCurrentTrajectoryPoint(pitch);
  tar_yaw_ = yaw;
}

void AccelAngleController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry&, bool)
{
  // Update timestamp
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // Horizontal acceleration & Attitude
  if (rcin.sub_mode)  // Translation mode
  {
    ax_filt_.setTargetPointAndUpdate(expoRemap(rcin.pitch, hor_acc_expo_, -max_hor_acc_, max_hor_acc_), dt);
    ay_filt_.setTargetPointAndUpdate(-expoRemap(rcin.roll, hor_acc_expo_, -max_hor_acc_, max_hor_acc_), dt);
    roll_filt_.setTargetPointAndUpdate(0., dt);
    pitch_filt_.setTargetPointAndUpdate(0., dt);
  }
  else  // Rotation mode
  {
    roll_filt_.setTargetPointAndUpdate(expoRemapDead(rcin.roll, atti_expo_, -max_attitude_, max_attitude_), dt);
    pitch_filt_.setTargetPointAndUpdate(expoRemapDead(rcin.pitch, atti_expo_, -max_attitude_, max_attitude_), dt);
    ax_filt_.setTargetPointAndUpdate(0., dt);
    ay_filt_.setTargetPointAndUpdate(0., dt);
  }

  // Vertical acceleration
  const auto az = expoRemap(rcin.throttle, ver_acc_expo_, -max_ver_acc_, max_ver_acc_);

  // Yaw
  const auto yawrate = expoRemapDead(rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_);
  tar_yaw_ += yawrate * dt;

  // Compute the acceleration wrt. the world frame
  const kdl::Vector tar_acc_G(ax_filt_.getTrajectoryPosition(), ay_filt_.getTrajectoryPosition(), az);
  const auto tar_acc_W = kdl::Rotation::RotZ(tar_yaw_) * tar_acc_G;

  // Publish commands
  publishAccel(rcin.header.stamp, tar_acc_W);
  publishAngle(rcin.header.stamp, roll_filt_.getTrajectoryPosition(), pitch_filt_.getTrajectoryPosition(), tar_yaw_);
}

void AccelAngleController::publishAccel(const builtin_interfaces::msg::Time& stamp, const kdl::Vector& acc)
{
  auto cmd = std::make_unique<tobas_command_msgs::Accel>();
  cmd->header.stamp = stamp;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
  cmd->accel = acc;

  accel_pub_->publish(std::move(cmd));
}

void AccelAngleController::publishAngle(const builtin_interfaces::msg::Time& stamp, double roll, double pitch, double yaw)
{
  auto cmd = std::make_unique<tobas_command_msgs::Angle>();
  cmd->header.stamp = stamp;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
  cmd->angle.set(roll, pitch, yaw);

  angle_pub_->publish(std::move(cmd));
}

bool AccelAngleController::maxHorizontalAccelCb(const double& p)
{
  max_hor_acc_ = p;
  return true;
}

bool AccelAngleController::maxHorizontalJerkCb(const double& p)
{
  ax_filt_.setMaxVelocity(p);
  ay_filt_.setMaxVelocity(p);
  return true;
}

bool AccelAngleController::maxVerticalAccelCb(const double& p)
{
  max_ver_acc_ = p;
  return true;
}

bool AccelAngleController::maxAttitudeCb(const double& p)
{
  max_attitude_ = st::deg2rad(p);
  return true;
}

bool AccelAngleController::maxAttitudeRateCb(const double& p)
{
  const auto max_atti_rate = st::deg2rad(p);  // [rad/s]
  roll_filt_.setMaxVelocity(max_atti_rate);
  pitch_filt_.setMaxVelocity(max_atti_rate);
  return true;
}

bool AccelAngleController::maxHeadingRateCb(const double& p)
{
  max_head_rate_ = st::deg2rad(p);
  return true;
}

bool AccelAngleController::horizontalAccelExpoCb(const double& p)
{
  hor_acc_expo_ = p / kExpoScale;
  return true;
}

bool AccelAngleController::verticalAccelExpoCb(const double& p)
{
  ver_acc_expo_ = p / kExpoScale;
  return true;
}

bool AccelAngleController::attitudeExpoCb(const double& p)
{
  atti_expo_ = p / kExpoScale;
  return true;
}

bool AccelAngleController::headingExpoCb(const double& p)
{
  head_expo_ = p / kExpoScale;
  return true;
}
}  // namespace rc
}  // namespace tobas

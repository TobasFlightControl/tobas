#include "tobas_rc_teleop/angle_throttle.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_constants/throttle.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas_rc_teleop
{
AngleThrottleController::AngleThrottleController()
{
}

bool AngleThrottleController::requirePosition()
{
  return false;
}

bool AngleThrottleController::requireVelocity()
{
  return false;
}

bool AngleThrottleController::requireAttitude()
{
  return true;
}

bool AngleThrottleController::requireHeading()
{
  return true;
}

void AngleThrottleController::initialize(tobas::BaseNode* node, tobas::FlightMode mode)
{
  node->addDynamicIntParam(addMode("max_attitude", mode), &self::maxAttitudeCb, this, 45, 0, 80, " deg");
  node->addDynamicIntParam(addMode("max_attitude_rate", mode), &self::maxAttitudeRateCb, this, 360, 0, 720, " dps");
  node->addDynamicIntParam(addMode("max_heading_rate", mode), &self::maxHeadingRateCb, this, 180, 0, 360, " dps");
  node->addDynamicIntParam(addMode("attitude_expo", mode), &self::attitudeExpoCb, this, -30, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("heading_expo", mode), &self::headingExpoCb, this, -15, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("throttle_expo", mode), &self::throttleExpoCb, this, 0, 0, kExpoScale);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::AngleThrottle>(tobas::topic::kAngleThrotCmd);
}

void AngleThrottleController::reset(const tobas_msgs::Odometry& odom, bool)
{
  t_last_rcin_ = odom.header.stamp;

  const auto [roll, pitch, yaw] = odom.frame.M.getRPY();
  roll_filt_.resetCurrentTrajectoryPoint(roll);
  pitch_filt_.resetCurrentTrajectoryPoint(pitch);
  tar_yaw_ = yaw;
}

void AngleThrottleController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry&, bool)
{
  // Update timestamp
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // Create a command
  auto cmd = std::make_unique<tobas_command_msgs::AngleThrottle>();
  cmd->header = rcin.header;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;

  // Roll
  roll_filt_.setTargetPosition(expoRemapDead(rcin.roll, atti_expo_, -max_attitude_, max_attitude_));
  roll_filt_.update(dt);
  cmd->angle.roll = roll_filt_.getTrajectoryPosition();

  // Pitch
  pitch_filt_.setTargetPosition(expoRemapDead(rcin.pitch, atti_expo_, -max_attitude_, max_attitude_));
  pitch_filt_.update(dt);
  cmd->angle.pitch = pitch_filt_.getTrajectoryPosition();

  // Yaw
  const auto yawrate = expoRemapDead(rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_);
  tar_yaw_ += yawrate * dt;
  cmd->angle.yaw = tar_yaw_;

  // Throttle
  cmd->throttle = expo(remap(rcin.throttle, tobas::kMinThrot, tobas::kMaxThrot), throt_expo_);

  // Publish the command
  cmd_pub_->publish(std::move(cmd));
}

bool AngleThrottleController::maxAttitudeCb(const long& p)
{
  max_attitude_ = tbs::deg2rad(p);
  return true;
}

bool AngleThrottleController::maxAttitudeRateCb(const long& p)
{
  const auto max_atti_rate = tbs::deg2rad(p);  // [rad/s]
  roll_filt_.setMaxVelocity(max_atti_rate);
  pitch_filt_.setMaxVelocity(max_atti_rate);
  return true;
}

bool AngleThrottleController::maxHeadingRateCb(const long& p)
{
  max_head_rate_ = tbs::deg2rad(p);
  return true;
}

bool AngleThrottleController::attitudeExpoCb(const long& p)
{
  atti_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool AngleThrottleController::headingExpoCb(const long& p)
{
  head_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool AngleThrottleController::throttleExpoCb(const long& p)
{
  throt_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}
}  // namespace tobas_rc_teleop

#include "tobas_rc_teleop/angle_throttle.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_constants/throttle.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace rc
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

void AngleThrottleController::initialize(BaseNode* node, FlightMode mode)
{
  node->addDynamicDoubleParam(addMode("max_attitude", mode), &self::maxAttitudeCb, this, 5., 9, 1, 16, " deg");
  node->addDynamicDoubleParam(addMode("max_attitude_rate", mode), &self::maxAttitudeRateCb, this, 45., 8, 1, 16, " dps");
  node->addDynamicDoubleParam(addMode("max_heading_rate", mode), &self::maxHeadingRateCb, this, 20., 9, 1, 18, " dps");
  node->addDynamicDoubleParam(addMode("attitude_expo", mode), &self::attitudeExpoCb, this, 5., -6, -20, 20);
  node->addDynamicDoubleParam(addMode("heading_expo", mode), &self::headingExpoCb, this, 5., -3, -20, 20);
  node->addDynamicDoubleParam(addMode("throttle_expo", mode), &self::throttleExpoCb, this, 5., 0, 0, 20);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::AngleThrottle>(topic::kAngleThrotCmd);
}

void AngleThrottleController::reset(const builtin_interfaces::msg::Time& stamp, const tobas_msgs::Odometry& setpoint, bool)
{
  t_last_rcin_ = stamp;

  const auto [roll, pitch, yaw] = setpoint.frame.M.getRPY();
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
  roll_filt_.setTargetPointAndUpdate(expoRemapDead(rcin.roll, atti_expo_, -max_attitude_, max_attitude_), dt);
  cmd->angle.roll = roll_filt_.getTrajectoryPosition();

  // Pitch
  pitch_filt_.setTargetPointAndUpdate(expoRemapDead(rcin.pitch, atti_expo_, -max_attitude_, max_attitude_), dt);
  cmd->angle.pitch = pitch_filt_.getTrajectoryPosition();

  // Yaw
  const auto yawrate = expoRemapDead(rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_);
  tar_yaw_ += yawrate * dt;
  cmd->angle.yaw = tar_yaw_;

  // Throttle
  cmd->throttle = expo(remap(rcin.throttle, kMinThrot, kMaxThrot), throt_expo_);

  // Publish the command
  cmd_pub_->publish(std::move(cmd));
}

bool AngleThrottleController::maxAttitudeCb(const double& p)
{
  max_attitude_ = st::deg2rad(p);
  return true;
}

bool AngleThrottleController::maxAttitudeRateCb(const double& p)
{
  const auto max_atti_rate = st::deg2rad(p);  // [rad/s]
  roll_filt_.setMaxVelocity(max_atti_rate);
  pitch_filt_.setMaxVelocity(max_atti_rate);
  return true;
}

bool AngleThrottleController::maxHeadingRateCb(const double& p)
{
  max_head_rate_ = st::deg2rad(p);
  return true;
}

bool AngleThrottleController::attitudeExpoCb(const double& p)
{
  atti_expo_ = p / kExpoScale;
  return true;
}

bool AngleThrottleController::headingExpoCb(const double& p)
{
  head_expo_ = p / kExpoScale;
  return true;
}

bool AngleThrottleController::throttleExpoCb(const double& p)
{
  throt_expo_ = p / kExpoScale;
  return true;
}
}  // namespace rc
}  // namespace tobas

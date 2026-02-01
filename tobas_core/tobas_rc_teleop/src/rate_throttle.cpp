#include "tobas_rc_teleop/rate_throttle.hpp"

#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas_rc_teleop
{
RateThrottleController::RateThrottleController()
{
}

bool RateThrottleController::requirePosition()
{
  return false;
}

bool RateThrottleController::requireVelocity()
{
  return false;
}

bool RateThrottleController::requireAttitude()
{
  return false;
}

bool RateThrottleController::requireHeading()
{
  return false;
}

void RateThrottleController::initialize(tobas::BaseNode* node, tobas::FlightMode mode)
{
  node->addDynamicIntParam(addMode("max_attitude_rate", mode), &self::maxAttitudeRateCb, this, 360, 0, 720, " dps");
  node->addDynamicIntParam(addMode("max_heading_rate", mode), &self::maxHeadingRateCb, this, 360, 0, 720, " dps");
  node->addDynamicIntParam(addMode("attitude_expo", mode), &self::attitudeExpoCb, this, -30, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("heading_expo", mode), &self::headingExpoCb, this, -15, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("throttle_expo", mode), &self::throttleExpoCb, this, 0, 0, kExpoScale);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::RateThrottle>(tobas::kRateThrotCmdTopic);
}

void RateThrottleController::reset(const tobas_msgs::Odometry&)
{
}

void RateThrottleController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry&)
{
  // コマンドを作成
  auto cmd = std::make_unique<tobas_command_msgs::RateThrottle>();
  cmd->header = rcin.header;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;
  cmd->rate.x(expoRemap(rcin.roll, atti_expo_, -max_atti_rate_, max_atti_rate_));
  cmd->rate.y(expoRemap(rcin.pitch, atti_expo_, -max_atti_rate_, max_atti_rate_));
  cmd->rate.z(expoRemap(rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_));
  cmd->throttle = expo(deadband(remap(rcin.throttle, tobas::kMinThrot, tobas::kMaxThrot)), throt_expo_);

  // コマンドを発行
  cmd_pub_->publish(std::move(cmd));
}

bool RateThrottleController::maxAttitudeRateCb(const long& p)
{
  max_atti_rate_ = tbs::deg2rad(p);
  return true;
}

bool RateThrottleController::maxHeadingRateCb(const long& p)
{
  max_head_rate_ = tbs::deg2rad(p);
  return true;
}

bool RateThrottleController::attitudeExpoCb(const long& p)
{
  atti_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool RateThrottleController::headingExpoCb(const long& p)
{
  head_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool RateThrottleController::throttleExpoCb(const long& p)
{
  throt_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}
}  // namespace tobas_rc_teleop

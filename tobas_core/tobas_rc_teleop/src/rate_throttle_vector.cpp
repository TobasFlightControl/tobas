#include "tobas_rc_teleop/rate_throttle_vector.hpp"

#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas_rc_teleop
{
RateThrottleVectorController::RateThrottleVectorController()
{
}

bool RateThrottleVectorController::requirePosition()
{
  return false;
}

bool RateThrottleVectorController::requireVelocity()
{
  return false;
}

bool RateThrottleVectorController::requireAttitude()
{
  return false;
}

bool RateThrottleVectorController::requireHeading()
{
  return false;
}

void RateThrottleVectorController::initialize(tobas::BaseNode* node, tobas::FlightMode mode)
{
  node->addDynamicIntParam(addMode("max_attitude_rate", mode), &self::maxAttitudeRateCb, this, 360, 0, 720, " dps");
  node->addDynamicIntParam(addMode("max_heading_rate", mode), &self::maxHeadingRateCb, this, 360, 0, 720, " dps");
  node->addDynamicIntParam(addMode("max_thrust_angle", mode), &self::maxThrustAngleCb, this, 90, 0, 180, " deg");
  node->addDynamicIntParam(addMode("attitude_expo", mode), &self::attitudeExpoCb, this, -30, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("heading_expo", mode), &self::headingExpoCb, this, -15, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("throttle_expo", mode), &self::throttleExpoCb, this, 0, 0, kExpoScale);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::RateThrottleVector>(tobas::kRateThrotCmdTopic);
}

void RateThrottleVectorController::reset(const tobas_msgs::Odometry&)
{
}

void RateThrottleVectorController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry&)
{
  auto cmd = std::make_unique<tobas_command_msgs::RateThrottleVector>();
  cmd->header = rcin.header;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;

  if (rcin.sub_mode) {  // 並進モード
    cmd->rate.y(0.);
    cmd->thrust_angle = remapDead(rcin.pitch, -max_thrust_angle_, max_thrust_angle_);
  }
  else {  // 回転モード
    cmd->rate.y(expoRemap(rcin.pitch, atti_expo_, -max_atti_rate_, max_atti_rate_));
    cmd->thrust_angle = 0.;
  }

  cmd->rate.x(expoRemap(rcin.roll, atti_expo_, -max_atti_rate_, max_atti_rate_));
  cmd->rate.z(expoRemap(rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_));
  cmd->throttle = expo(remap(rcin.throttle, tobas::kMinThrot, tobas::kMaxThrot), throt_expo_);

  cmd_pub_->publish(std::move(cmd));
}

bool RateThrottleVectorController::maxAttitudeRateCb(const long& p)
{
  max_atti_rate_ = tbs::deg2rad(p);
  return true;
}

bool RateThrottleVectorController::maxHeadingRateCb(const long& p)
{
  max_head_rate_ = tbs::deg2rad(p);
  return true;
}

bool RateThrottleVectorController::maxThrustAngleCb(const long& p)
{
  max_thrust_angle_ = tbs::deg2rad(p);
  return true;
}

bool RateThrottleVectorController::attitudeExpoCb(const long& p)
{
  atti_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool RateThrottleVectorController::headingExpoCb(const long& p)
{
  head_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool RateThrottleVectorController::throttleExpoCb(const long& p)
{
  throt_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}
}  // namespace tobas_rc_teleop

#include <tobas_ros2_tools/time.hpp>

#include "../include/tobas_rc_teleop/rate_throttle.hpp"

using namespace std;

namespace tobas_rc_teleop
{
RateThrottleController::RateThrottleController()
{
}

bool RateThrottleController::requirePosition()
{
  return false;
}

bool RateThrottleController::requireOrientation()
{
  return false;
}

bool RateThrottleController::requireLinearVelocity()
{
  return false;
}

bool RateThrottleController::requireAngularVelocity()
{
  return true;
}

void RateThrottleController::initialize(tobas::BaseNode* node, tobas::flight_mode_t mode)
{
  node->addDynamicDoubleParam(addMode("max_attitude_rate", mode), &self::maxAttitudeRateCb, this, M_PI, 0., M_PI * 2);
  node->addDynamicDoubleParam(addMode("max_heading_rate", mode), &self::maxHeadingRateCb, this, M_PI_2, 0., M_PI * 2);
  node->addDynamicIntParam(addMode("attitude_expo", mode), &self::attitudeExpoCb, this, 0, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("heading_expo", mode), &self::headingExpoCb, this, 0, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("throttle_expo", mode), &self::throttleExpoCb, this, 0, 0, kExpoScale);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::RateThrottle>(tobas::kRateThrottleCmdTopic);
}

void RateThrottleController::reset(const tobas_msgs::Odometry&)
{
}

void RateThrottleController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry&)
{
  // コマンドを作成
  auto cmd = make_unique<tobas_command_msgs::RateThrottle>();
  cmd->header = rcin.header;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;
  cmd->rate.x(expoRemap(rcin.roll, atti_expo_, -max_atti_rate_, max_atti_rate_));
  cmd->rate.y(expoRemap(rcin.pitch, atti_expo_, -max_atti_rate_, max_atti_rate_));
  cmd->rate.z(expoRemap(rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_));
  cmd->throttle = expo(remap(rcin.throttle, tobas::kMinThrot, tobas::kMaxThrot), throt_expo_);

  // コマンドを発行
  cmd_pub_->publish(move(cmd));
}

bool RateThrottleController::maxAttitudeRateCb(const double& p)
{
  max_atti_rate_ = p;
  return true;
}

bool RateThrottleController::maxHeadingRateCb(const double& p)
{
  max_head_rate_ = p;
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

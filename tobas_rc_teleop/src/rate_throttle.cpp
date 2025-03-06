#include <tobas_ros2_tools/time.hpp>

#include "../include/tobas_rc_teleop/rate_throttle.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

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

void RateThrottleController::initialize(tobas::BaseNode* node)
{
  getStaticRosParams(node);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::RateThrottle>(tobas::kRateThrottleCmdTopic);
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
  cmd->rate.x(remap(rcin.roll, -max_atti_rate_, max_atti_rate_));
  cmd->rate.y(remap(rcin.pitch, -max_atti_rate_, max_atti_rate_));
  cmd->rate.z(remap(rcin.yaw, -max_head_rate_, max_head_rate_));
  cmd->throttle = remap(rcin.throttle, tobas::kMinThrot, tobas::kMaxThrot);

  // コマンドを発行
  cmd_pub_->publish(move(cmd));
}

void RateThrottleController::getStaticRosParams(tobas::BaseNode* node)
{
  max_atti_rate_ = node->getDoubleParam("max_attitude_rate", kDefaultMaxAttitudeRate);
  if (max_atti_rate_ < 0)
  {
    node->error("Maximum attitude rate must be positive.");
    max_atti_rate_ = kDefaultMaxAttitudeRate;
  }

  max_head_rate_ = node->getDoubleParam("max_heading_rate", kDefaultMaxHeadingRate);
  if (max_head_rate_ < 0)
  {
    node->error("Maximum heading rate must be positive.");
    max_head_rate_ = kDefaultMaxHeadingRate;
  }
}
}  // namespace tobas_rc_teleop

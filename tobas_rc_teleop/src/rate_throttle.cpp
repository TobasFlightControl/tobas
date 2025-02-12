#include <tobas_ros2_tools/time.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_rc_teleop/rate_throttle.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;

namespace tobas_rc_teleop
{
RateThrottleController::RateThrottleController()
{
}

void RateThrottleController::initialize(tobas::BaseNode* node)
{
  getStaticRosParams(node);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::msg::RateThrottle>(tobas::kRateThrottleCmdTopic);
}

void RateThrottleController::reset(const tobas_msgs::Odometry&)
{
}

void RateThrottleController::update(const tobas_msgs::msg::RCInput& rcin, const tobas_msgs::Odometry&)
{
  // コマンドを作成
  auto cmd = std::make_unique<tobas_command_msgs::msg::RateThrottle>();
  cmd->header = rcin.header;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;
  cmd->droll = remap(rcin.roll, -max_attitude_rate_, max_attitude_rate_);
  cmd->dpitch = remap(rcin.pitch, -max_attitude_rate_, max_attitude_rate_);
  cmd->dyaw = remap(rcin.yaw, -max_heading_rate_, max_heading_rate_);
  cmd->throttle = remap(rcin.throttle, tobas::kMinThrot, tobas::kMaxThrot);

  // コマンドを発行
  cmd_pub_->publish(move(cmd));
}

void RateThrottleController::getStaticRosParams(tobas::BaseNode* node)
{
  max_attitude_rate_ = node->getDoubleParam("max_attitude_rate", kDefaultMaxAttitudeRate);
  if (max_attitude_rate_ < 0)
  {
    node->error("Maximum attitude rate must be positive.");
    max_attitude_rate_ = kDefaultMaxAttitudeRate;
  }

  max_heading_rate_ = node->getDoubleParam("max_heading_rate", kDefaultMaxHeadingRate);
  if (max_heading_rate_ < 0)
  {
    node->error("Maximum heading rate must be positive.");
    max_heading_rate_ = kDefaultMaxHeadingRate;
  }
}
}  // namespace tobas_rc_teleop

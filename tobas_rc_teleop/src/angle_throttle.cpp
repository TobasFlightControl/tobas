#include <tobas_ros2_tools/time.hpp>

#include "../include/tobas_rc_teleop/angle_throttle.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;

namespace tobas_rc_teleop
{
AngleThrottleController::AngleThrottleController()
{
}

bool AngleThrottleController::requirePosition()
{
  return false;
}

bool AngleThrottleController::requireOrientation()
{
  return true;
}

bool AngleThrottleController::requireLinearVelocity()
{
  return false;
}

bool AngleThrottleController::requireAngularVelocity()
{
  return false;
}

void AngleThrottleController::initialize(tobas::BaseNode* node)
{
  getStaticRosParams(node);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::AngleThrottle>(tobas::kAngleThrottleCmdTopic);
}

void AngleThrottleController::reset(const tobas_msgs::Odometry& odom)
{
  yaw_ = odom.frame.M.getYaw();
  t_last_rcin_ = odom.header.stamp;
}

void AngleThrottleController::update(const tobas_msgs::msg::RCInput& rcin, const tobas_msgs::Odometry&)
{
  // 時刻を更新
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // ヨーの目標値を更新
  const auto yawrate = remapDead(rcin.yaw, -max_head_rate_, max_head_rate_);
  yaw_ += yawrate * dt;

  // コマンドを作成
  auto cmd = std::make_unique<tobas_command_msgs::AngleThrottle>();
  cmd->header = rcin.header;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;

  // 姿勢とスロットルを埋める
  cmd->angle.roll = remap(rcin.roll, -max_attitude_, max_attitude_);
  cmd->angle.pitch = remap(rcin.pitch, -max_attitude_, max_attitude_);
  cmd->angle.yaw = yaw_;
  cmd->throttle = remap(rcin.throttle, tobas::kMinThrot, tobas::kMaxThrot);

  // コマンドを発行
  cmd_pub_->publish(move(cmd));
}

void AngleThrottleController::getStaticRosParams(tobas::BaseNode* node)
{
  max_attitude_ = node->getDoubleParam("max_attitude", kDefaultMaxAttitude);
  if (max_attitude_ < 0)
  {
    node->error("Maximum attitude angle must be positive.");
    max_attitude_ = kDefaultMaxAttitude;
  }

  max_head_rate_ = node->getDoubleParam("max_heading_rate", kDefaultMaxHeadingRate);
  if (max_head_rate_ < 0)
  {
    node->error("Maximum heading rate must be positive.");
    max_head_rate_ = kDefaultMaxHeadingRate;
  }
}
}  // namespace tobas_rc_teleop

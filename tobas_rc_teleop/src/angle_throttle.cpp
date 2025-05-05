#include "../include/tobas_rc_teleop/angle_throttle.hpp"

#include <tobas_ros2_tools/time.hpp>

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

void AngleThrottleController::initialize(tobas::BaseNode* node, tobas::flight_mode_t mode)
{
  node->addDynamicDoubleParam(addMode("max_attitude", mode), &self::maxAttitudeCb, this, M_PI / 4, 0., M_PI_2);
  node->addDynamicDoubleParam(addMode("max_heading_rate", mode), &self::maxHeadingRateCb, this, M_PI_2, 0., M_PI * 2);
  node->addDynamicIntParam(addMode("attitude_expo", mode), &self::attitudeExpoCb, this, 0, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("heading_expo", mode), &self::headingExpoCb, this, 0, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("throttle_expo", mode), &self::throttleExpoCb, this, 0, 0, kExpoScale);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::AngleThrottle>(tobas::kAngleThrottleCmdTopic);
}

void AngleThrottleController::reset(const tobas_msgs::Odometry& odom)
{
  yaw_ = odom.frame.M.getYaw();
  t_last_rcin_ = odom.header.stamp;
}

void AngleThrottleController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry&)
{
  // 時刻を更新
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // ヨーの目標値を更新
  const auto yawrate = expoRemapDead(rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_);
  yaw_ += yawrate * dt;

  // コマンドを作成
  auto cmd = make_unique<tobas_command_msgs::AngleThrottle>();
  cmd->header = rcin.header;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;

  // 姿勢とスロットルを埋める
  cmd->angle.roll = expoRemap(rcin.roll, atti_expo_, -max_attitude_, max_attitude_);
  cmd->angle.pitch = expoRemap(rcin.pitch, atti_expo_, -max_attitude_, max_attitude_);
  cmd->angle.yaw = yaw_;
  cmd->throttle = expo(remap(rcin.throttle, tobas::kMinThrot, tobas::kMaxThrot), throt_expo_);

  // コマンドを発行
  cmd_pub_->publish(move(cmd));
}

bool AngleThrottleController::maxAttitudeCb(const double& p)
{
  max_attitude_ = p;
  return true;
}

bool AngleThrottleController::maxHeadingRateCb(const double& p)
{
  max_head_rate_ = p;
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

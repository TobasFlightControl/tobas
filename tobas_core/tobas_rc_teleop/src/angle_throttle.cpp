#include "tobas_rc_teleop/angle_throttle.hpp"

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
  node->addDynamicIntParam(addMode("max_heading_rate", mode), &self::maxHeadingRateCb, this, 180, 0, 360, " dps");
  node->addDynamicIntParam(addMode("attitude_expo", mode), &self::attitudeExpoCb, this, -30, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("heading_expo", mode), &self::headingExpoCb, this, -15, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("throttle_expo", mode), &self::throttleExpoCb, this, 0, 0, kExpoScale);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::AngleThrottle>(tobas::kAngleThrotCmdTopic);
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
  auto cmd = std::make_unique<tobas_command_msgs::AngleThrottle>();
  cmd->header = rcin.header;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;

  // 姿勢とスロットルを埋める
  cmd->angle.roll = expoRemapDead(rcin.roll, atti_expo_, -max_attitude_, max_attitude_);
  cmd->angle.pitch = expoRemapDead(rcin.pitch, atti_expo_, -max_attitude_, max_attitude_);
  cmd->angle.yaw = yaw_;
  cmd->throttle = expo(remap(rcin.throttle, tobas::kMinThrot, tobas::kMaxThrot), throt_expo_);

  // コマンドを発行
  cmd_pub_->publish(std::move(cmd));
}

bool AngleThrottleController::maxAttitudeCb(const long& p)
{
  max_attitude_ = tbs::deg2rad(p);
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

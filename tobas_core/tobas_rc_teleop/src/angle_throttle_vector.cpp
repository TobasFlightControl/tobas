#include "tobas_rc_teleop/angle_throttle_vector.hpp"

#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas_rc_teleop
{
AngleThrottleVectorController::AngleThrottleVectorController()
{
}

bool AngleThrottleVectorController::requirePosition()
{
  return false;
}

bool AngleThrottleVectorController::requireVelocity()
{
  return false;
}

bool AngleThrottleVectorController::requireAttitude()
{
  return true;
}

bool AngleThrottleVectorController::requireHeading()
{
  return true;
}

void AngleThrottleVectorController::initialize(tobas::BaseNode* node, tobas::FlightMode mode)
{
  node->addDynamicIntParam(addMode("max_attitude", mode), &self::maxAttitudeCb, this, 45, 0, 80, " deg");
  node->addDynamicIntParam(addMode("max_heading_rate", mode), &self::maxHeadingRateCb, this, 180, 0, 360, " dps");
  node->addDynamicIntParam(addMode("max_thrust_angle", mode), &self::maxThrustAngleCb, this, 90, 0, 180, " deg");
  node->addDynamicIntParam(addMode("attitude_expo", mode), &self::attitudeExpoCb, this, -30, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("heading_expo", mode), &self::headingExpoCb, this, -15, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("throttle_expo", mode), &self::throttleExpoCb, this, 0, 0, kExpoScale);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::AngleThrottleVector>(tobas::kAngleThrotCmdTopic);
}

void AngleThrottleVectorController::reset(const tobas_msgs::Odometry& odom)
{
  yaw_ = odom.frame.M.getYaw();
  t_last_rcin_ = odom.header.stamp;
}

void AngleThrottleVectorController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry&)
{
  // 時刻を更新
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // ヨーの目標値を更新
  const auto yawrate = expoRemapDead(rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_);
  yaw_ += yawrate * dt;

  // コマンドを作成
  auto cmd = std::make_unique<tobas_command_msgs::AngleThrottleVector>();
  cmd->header = rcin.header;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;

  // コマンドのモードに依らない部分を埋める
  cmd->angle.roll = expoRemapDead(rcin.roll, atti_expo_, -max_attitude_, max_attitude_);
  cmd->angle.yaw = yaw_;
  cmd->throttle = expo(remap(rcin.throttle, tobas::kMinThrot, tobas::kMaxThrot), throt_expo_);

  // コマンドのサブモードに依存する要素を埋める
  if (rcin.sub_mode)  // 並進モード
  {
    cmd->angle.pitch = 0.;
    cmd->thrust_angle = remapDead(rcin.pitch, -max_thrust_angle_, max_thrust_angle_);
  }
  else  // 回転モード
  {
    cmd->angle.pitch = expoRemapDead(rcin.pitch, atti_expo_, -max_attitude_, max_attitude_);
    cmd->thrust_angle = 0.;
  }

  // コマンドを発行
  cmd_pub_->publish(std::move(cmd));
}

bool AngleThrottleVectorController::maxAttitudeCb(const long& p)
{
  max_attitude_ = tbs::deg2rad(p);
  return true;
}

bool AngleThrottleVectorController::maxHeadingRateCb(const long& p)
{
  max_head_rate_ = tbs::deg2rad(p);
  return true;
}

bool AngleThrottleVectorController::maxThrustAngleCb(const long& p)
{
  max_thrust_angle_ = tbs::deg2rad(p);
  return true;
}

bool AngleThrottleVectorController::attitudeExpoCb(const long& p)
{
  atti_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool AngleThrottleVectorController::headingExpoCb(const long& p)
{
  head_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool AngleThrottleVectorController::throttleExpoCb(const long& p)
{
  throt_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}
}  // namespace tobas_rc_teleop

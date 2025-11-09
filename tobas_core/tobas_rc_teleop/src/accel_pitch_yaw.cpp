#include "tobas_rc_teleop/accel_pitch_yaw.hpp"

#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas_rc_teleop
{
AccelPitchYawController::AccelPitchYawController()
{
}

bool AccelPitchYawController::requirePosition()
{
  return false;
}

bool AccelPitchYawController::requireOrientation()
{
  return true;
}

bool AccelPitchYawController::requireLinearVelocity()
{
  return false;
}

bool AccelPitchYawController::requireAngularVelocity()
{
  return true;
}

void AccelPitchYawController::initialize(tobas::BaseNode* node, tobas::FlightMode mode)
{
  node->addDynamicDoubleParam(
    addMode("max_horizontal_accel", mode), &self::maxHorizontalAccelCb, this, 0.5, 10, 4, 20, " m/s^2");
  node->addDynamicDoubleParam(
    addMode("max_vertical_accel", mode), &self::maxVerticalAccelCb, this, 0.5, 8, 4, 20, " m/s^2");
  node->addDynamicIntParam(addMode("max_attitude", mode), &self::maxAttitudeCb, this, 90, 0, 180, " deg");
  node->addDynamicIntParam(addMode("max_heading_rate", mode), &self::maxHeadingRateCb, this, 90, 0, 360, " dps");
  node->addDynamicIntParam(
    addMode("horizontal_accel_expo", mode), &self::horizontalAccelExpoCb, this, -30, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(
    addMode("vertical_accel_expo", mode), &self::verticalAccelExpoCb, this, 0, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("attitude_expo", mode), &self::attitudeExpoCb, this, 0, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("heading_expo", mode), &self::headingExpoCb, this, -15, -kExpoScale, kExpoScale);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::AccelPitchYaw>(tobas::kAccelPitchYawCmdTopic);
}

void AccelPitchYawController::reset(const tobas_msgs::Odometry& odom)
{
  t_last_rcin_ = odom.header.stamp;
  tar_acc_G_.setZero();
  tar_pitch_ = odom.frame.M.getPitch();
  tar_yaw_ = odom.frame.M.getYaw();
}

void AccelPitchYawController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry&)
{
  // 時刻を更新
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // サブモードで並進制御モードと回転制御モードを切り替える
  if (rcin.sub_mode)  // 回転固定で並進制御
  {
    tar_acc_G_.x(expoRemap(rcin.pitch, hor_acc_expo_, -max_hor_acc_, max_hor_acc_));
    tar_pitch_ = 0.;
  }
  else  // 並進固定で回転制御
  {
    tar_acc_G_.x(0.);
    tar_pitch_ = expoRemapDead(rcin.pitch, atti_expo_, -max_attitude_, max_attitude_);
  }

  // Y, Z, Yaw はサブモードに依らない
  tar_acc_G_.y(-expoRemap(rcin.roll, hor_acc_expo_, -max_hor_acc_, max_hor_acc_));
  tar_acc_G_.z(expoRemap(rcin.throttle, ver_acc_expo_, -max_ver_acc_, max_ver_acc_));
  const auto yawrate = expoRemapDead(rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_);

  // 目標加速度を地面座標系から世界座標系に変換
  // ヨー角の現在値で変換すると直進指令でも進路が曲がってしまうため，指令値で変換する．
  const auto tar_acc_W = kdl::Rotation::RotZ(tar_yaw_) * tar_acc_G_;

  // ヨーレートを積分
  tar_yaw_ += yawrate * dt;

  // コマンドを作成
  auto cmd = std::make_unique<tobas_command_msgs::AccelPitchYaw>();
  cmd->header = rcin.header;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;
  cmd->accel = tar_acc_W;
  cmd->pitch = tar_pitch_;
  cmd->yaw = tar_yaw_;

  // コマンドを発行
  cmd_pub_->publish(std::move(cmd));
}

bool AccelPitchYawController::maxHorizontalAccelCb(const double& p)
{
  max_hor_acc_ = p;
  return true;
}

bool AccelPitchYawController::maxVerticalAccelCb(const double& p)
{
  max_ver_acc_ = p;
  return true;
}

bool AccelPitchYawController::maxAttitudeCb(const long& p)
{
  max_attitude_ = tbs::deg2rad(p);
  return true;
}

bool AccelPitchYawController::maxHeadingRateCb(const long& p)
{
  max_head_rate_ = tbs::deg2rad(p);
  return true;
}

bool AccelPitchYawController::horizontalAccelExpoCb(const long& p)
{
  hor_acc_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool AccelPitchYawController::verticalAccelExpoCb(const long& p)
{
  ver_acc_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool AccelPitchYawController::attitudeExpoCb(const long& p)
{
  atti_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool AccelPitchYawController::headingExpoCb(const long& p)
{
  head_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}
}  // namespace tobas_rc_teleop

#include <tobas_ros2_tools/time.hpp>

#include "../include/tobas_rc_teleop/accel_angle.hpp"

using namespace std;

namespace tobas_rc_teleop
{
AccelAngleController::AccelAngleController()
{
}

bool AccelAngleController::requirePosition()
{
  return false;
}

bool AccelAngleController::requireOrientation()
{
  return true;
}

bool AccelAngleController::requireLinearVelocity()
{
  return false;
}

bool AccelAngleController::requireAngularVelocity()
{
  return true;
}

void AccelAngleController::initialize(tobas::BaseNode* node, tobas::flight_mode_t mode)
{
  node->addDynamicDoubleParam(addMode("max_horizontal_accel", mode), &self::maxHorizontalAccelCb, this, 3., 0., 10.);
  node->addDynamicDoubleParam(addMode("max_vertical_accel", mode), &self::maxVerticalAccelCb, this, 2., 0., 10.);
  node->addDynamicDoubleParam(addMode("max_attitude", mode), &self::maxAttitudeCb, this, M_PI_2, 0., M_PI);
  node->addDynamicDoubleParam(addMode("max_heading_rate", mode), &self::maxHeadingRateCb, this, M_PI_2, 0., M_PI * 2);
  node->addDynamicIntParam(
    addMode("horizontal_accel_expo", mode), &self::horizontalAccelExpoCb, this, 0, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(
    addMode("vertical_accel_expo", mode), &self::verticalAccelExpoCb, this, 0, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("attitude_expo", mode), &self::attitudeExpoCb, this, 0, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("heading_expo", mode), &self::headingExpoCb, this, 0, -kExpoScale, kExpoScale);

  accel_pub_ = node->createPublisher<tobas_command_msgs::Accel>(tobas::kAccelCmdTopic);
  angle_pub_ = node->createPublisher<tobas_command_msgs::Angle>(tobas::kAngleCmdTopic);
}

void AccelAngleController::reset(const tobas_msgs::Odometry& odom)
{
  t_last_rcin_ = odom.header.stamp;
  tar_acc_G_.setZero();
  odom.frame.M.getRPY(tar_angle_.roll, tar_angle_.pitch, tar_angle_.yaw);
}

void AccelAngleController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry&)
{
  // 時刻を更新
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // サブモードで並進制御モードと回転制御モードを切り替える
  if (rcin.sub_mode)  // 回転固定で並進制御
  {
    // RC入力から目標水平加速度を計算
    tar_acc_G_.x(expoRemap(rcin.pitch, hor_acc_expo_, -max_hor_acc_, max_hor_acc_));
    tar_acc_G_.y(-expoRemap(rcin.roll, hor_acc_expo_, -max_hor_acc_, max_hor_acc_));

    // 目標姿勢角はゼロ
    tar_angle_.roll = 0.;
    tar_angle_.pitch = 0.;
  }
  else  // 並進固定で回転制御
  {
    // RC入力から目標姿勢を計算
    tar_angle_.roll = expoRemapDead(rcin.roll, atti_expo_, -max_attitude_, max_attitude_);
    tar_angle_.pitch = expoRemapDead(rcin.pitch, atti_expo_, -max_attitude_, max_attitude_);

    // 目標水平加速度はゼロ
    tar_acc_G_.x(0.);
    tar_acc_G_.y(0.);
  }

  // RC入力から鉛直加速度とヨーレートを計算
  tar_acc_G_.z(expoRemap(rcin.throttle, ver_acc_expo_, -max_ver_acc_, max_ver_acc_));
  const auto yawrate = expoRemapDead(rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_);

  // 目標加速度を地面座標系から世界座標系に変換
  // ヨー角の現在値で変換すると直進指令でも進路が曲がってしまうため，指令値で変換する．
  const auto tar_acc_W = kdl::Rotation::RotZ(tar_angle_.yaw) * tar_acc_G_;

  // ヨーレートを積分
  tar_angle_.yaw += yawrate * dt;

  // コマンドを発行
  publishAccel(rcin.header.stamp, tar_acc_W);
  publishAngle(rcin.header.stamp, tar_angle_);
}

void AccelAngleController::publishAccel(const builtin_interfaces::msg::Time& stamp, const kdl::Vector& acc)
{
  auto cmd = make_unique<tobas_command_msgs::Accel>();
  cmd->header.stamp = stamp;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;
  cmd->accel = acc;

  accel_pub_->publish(move(cmd));
}

void AccelAngleController::publishAngle(const builtin_interfaces::msg::Time& stamp, const kdl::Euler& angle)
{
  auto cmd = make_unique<tobas_command_msgs::Angle>();
  cmd->header.stamp = stamp;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;
  cmd->angle = angle;

  angle_pub_->publish(move(cmd));
}

bool AccelAngleController::maxHorizontalAccelCb(const double& p)
{
  max_hor_acc_ = p;
  return true;
}

bool AccelAngleController::maxVerticalAccelCb(const double& p)
{
  max_ver_acc_ = p;
  return true;
}

bool AccelAngleController::maxAttitudeCb(const double& p)
{
  max_attitude_ = p;
  return true;
}

bool AccelAngleController::maxHeadingRateCb(const double& p)
{
  max_head_rate_ = p;
  return true;
}

bool AccelAngleController::horizontalAccelExpoCb(const long& p)
{
  hor_acc_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool AccelAngleController::verticalAccelExpoCb(const long& p)
{
  ver_acc_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool AccelAngleController::attitudeExpoCb(const long& p)
{
  atti_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool AccelAngleController::headingExpoCb(const long& p)
{
  head_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}
}  // namespace tobas_rc_teleop

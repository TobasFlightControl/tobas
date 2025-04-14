#include <tobas_ros2_tools/time.hpp>

#include "../include/tobas_rc_teleop/accel_rate.hpp"

using namespace std;

namespace tobas_rc_teleop
{
AccelRateController::AccelRateController()
{
}

bool AccelRateController::requirePosition()
{
  return false;
}

bool AccelRateController::requireOrientation()
{
  return true;
}

bool AccelRateController::requireLinearVelocity()
{
  return false;
}

bool AccelRateController::requireAngularVelocity()
{
  return true;
}

void AccelRateController::initialize(tobas::BaseNode* node, tobas::flight_mode_t mode)
{
  node->addDynamicDoubleParam(addMode("max_horizontal_accel", mode), &self::maxHorizontalAccelCb, this, 3., 0., 10.);
  node->addDynamicDoubleParam(addMode("max_vertical_accel", mode), &self::maxVerticalAccelCb, this, 2., 0., 10.);
  node->addDynamicDoubleParam(addMode("max_attitude_rate", mode), &self::maxAttitudeRateCb, this, M_PI, 0., M_PI * 2);
  node->addDynamicDoubleParam(addMode("max_heading_rate", mode), &self::maxHeadingRateCb, this, M_PI_2, 0., M_PI * 2);

  accel_pub_ = node->createPublisher<tobas_command_msgs::Accel>(tobas::kAccelCmdTopic);
  rate_pub_ = node->createPublisher<tobas_command_msgs::Rate>(tobas::kRateCmdTopic);
}

void AccelRateController::reset(const tobas_msgs::Odometry&)
{
  tar_acc_G_.setZero();
  tar_gyro_B_.setZero();
}

void AccelRateController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom)
{
  // サブモードで並進制御モードと回転制御モードを切り替える
  if (rcin.sub_mode)  // 回転固定で並進制御
  {
    // RC入力から目標水平加速度を計算
    tar_acc_G_.x(remap(rcin.pitch, -max_hor_acc_, max_hor_acc_));
    tar_acc_G_.y(-remap(rcin.roll, -max_hor_acc_, max_hor_acc_));

    // 目標角速度はゼロ
    tar_gyro_B_.x(0.);
    tar_gyro_B_.y(0.);
  }
  else  // 並進固定で回転制御
  {
    // RC入力から目標角速度を計算
    tar_gyro_B_.x(remap(rcin.roll, -max_atti_rate_, max_atti_rate_));
    tar_gyro_B_.y(remap(rcin.pitch, -max_atti_rate_, max_atti_rate_));

    // 目標水平加速度はゼロ
    tar_acc_G_.x(0.);
    tar_acc_G_.y(0.);
  }

  // RC入力から鉛直速度とヨーレートを計算
  tar_acc_G_.z(remap(rcin.throttle, -max_ver_acc_, max_ver_acc_));
  tar_gyro_B_.z(remap(rcin.yaw, -max_head_rate_, max_head_rate_));

  // 目標加速度を地面座標系から世界座標系に変換
  const auto cur_yaw = odom.frame.M.getYaw();
  const auto tar_acc_W = kdl::Rotation::RotZ(cur_yaw) * tar_acc_G_;

  // コマンドを発行
  publishAccel(rcin.header.stamp, tar_acc_W);
  publishRate(rcin.header.stamp, tar_gyro_B_);
}

void AccelRateController::publishAccel(const builtin_interfaces::msg::Time& stamp, const kdl::Vector& acc)
{
  auto cmd = make_unique<tobas_command_msgs::Accel>();
  cmd->header.stamp = stamp;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;
  cmd->accel = acc;

  accel_pub_->publish(move(cmd));
}

void AccelRateController::publishRate(const builtin_interfaces::msg::Time& stamp, const kdl::Vector& rate)
{
  auto cmd = make_unique<tobas_command_msgs::Rate>();
  cmd->header.stamp = stamp;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;
  cmd->rate = rate;

  rate_pub_->publish(move(cmd));
}

bool AccelRateController::maxHorizontalAccelCb(const double& p)
{
  max_hor_acc_ = p;
  return true;
}

bool AccelRateController::maxVerticalAccelCb(const double& p)
{
  max_ver_acc_ = p;
  return true;
}

bool AccelRateController::maxAttitudeRateCb(const double& p)
{
  max_atti_rate_ = p;
  return true;
}

bool AccelRateController::maxHeadingRateCb(const double& p)
{
  max_head_rate_ = p;
  return true;
}
}  // namespace tobas_rc_teleop

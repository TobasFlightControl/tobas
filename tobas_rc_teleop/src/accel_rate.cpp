#include <tobas_ros2_tools/time.hpp>

#include "../include/tobas_rc_teleop/accel_rate.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

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

void AccelRateController::initialize(tobas::BaseNode* node)
{
  getStaticRosParams(node);

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
  // GPSwの状態によって並進制御モードと回転制御モードを切り替える
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

void AccelRateController::getStaticRosParams(tobas::BaseNode* node)
{
  max_hor_acc_ = node->getDoubleParam("max_horizontal_accel", kDefaultMaxHorAcc);
  if (max_hor_acc_ < 0)
  {
    node->error("Maximum horizontal velocity must be positive.");
    max_hor_acc_ = kDefaultMaxHorAcc;
  }

  max_ver_acc_ = node->getDoubleParam("max_vertical_accel", kDefaultMaxVerAcc);
  if (max_ver_acc_ < 0)
  {
    node->error("Maximum vertical velocity must be positive.");
    max_ver_acc_ = kDefaultMaxVerAcc;
  }

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

void AccelRateController::publishAccel(const builtin_interfaces::msg::Time& stamp, const kdl::Vector& acc)
{
  auto cmd = std::make_unique<tobas_command_msgs::Accel>();
  cmd->header.stamp = stamp;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;
  cmd->accel = acc;

  accel_pub_->publish(move(cmd));
}

void AccelRateController::publishRate(const builtin_interfaces::msg::Time& stamp, const kdl::Vector& rate)
{
  auto cmd = std::make_unique<tobas_command_msgs::Rate>();
  cmd->header.stamp = stamp;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;
  cmd->rate = rate;

  rate_pub_->publish(move(cmd));
}
}  // namespace tobas_rc_teleop

#include <tobas_ros2_tools/time.hpp>

#include "../include/tobas_rc_teleop/accel_angle.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

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

void AccelAngleController::initialize(tobas::BaseNode* node)
{
  getStaticRosParams(node);

  accel_pub_ = node->createPublisher<tobas_command_msgs::Accel>(tobas::kAccelCmdTopic);
  angle_pub_ = node->createPublisher<tobas_command_msgs::Angle>(tobas::kAngleCmdTopic);
}

void AccelAngleController::reset(const tobas_msgs::Odometry& odom)
{
  t_last_rcin_ = odom.header.stamp;
  tar_acc_G_.setZero();
  odom.frame.M.getRPY(tar_angle_.roll, tar_angle_.pitch, tar_angle_.yaw);
}

void AccelAngleController::update(const tobas_msgs::msg::RCInput& rcin, const tobas_msgs::Odometry&)
{
  // 時刻を更新
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // GPSwの状態によって並進制御モードと回転制御モードを切り替える
  if (rcin.gpsw)  // 回転固定で並進制御
  {
    // RC入力から目標水平加速度を計算
    tar_acc_G_.x(remap(rcin.pitch, -max_hor_acc_, max_hor_acc_));
    tar_acc_G_.y(-remap(rcin.roll, -max_hor_acc_, max_hor_acc_));

    // 目標姿勢角はゼロ
    tar_angle_.roll = 0.;
    tar_angle_.pitch = 0.;
  }
  else  // 並進固定で回転制御
  {
    // RC入力から目標姿勢を計算
    tar_angle_.roll = remapDead(rcin.roll, -max_attitude_, max_attitude_);
    tar_angle_.pitch = remapDead(rcin.pitch, -max_attitude_, max_attitude_);

    // 目標水平加速度はゼロ
    tar_acc_G_.x(0.);
    tar_acc_G_.y(0.);
  }

  // RC入力から鉛直速度とヨーレートを計算
  tar_acc_G_.z(remap(rcin.throttle, -max_ver_acc_, max_ver_acc_));
  const auto yawrate = remapDead(rcin.yaw, -max_head_rate_, max_head_rate_);

  // 目標加速度を地面座標系から世界座標系に変換
  // ヨー角の現在値で変換すると直進指令でも進路が曲がってしまうため，指令値で変換する．
  const auto tar_acc_W = kdl::Rotation::RotZ(tar_angle_.yaw) * tar_acc_G_;

  // ヨーレートを積分
  tar_angle_.yaw += yawrate * dt;

  // コマンドを発行
  publishAccel(rcin.header.stamp, tar_acc_W);
  publishAngle(rcin.header.stamp, tar_angle_);
}

void AccelAngleController::getStaticRosParams(tobas::BaseNode* node)
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

void AccelAngleController::publishAccel(const builtin_interfaces::msg::Time& stamp, const kdl::Vector& acc)
{
  auto cmd = std::make_unique<tobas_command_msgs::Accel>();
  cmd->header.stamp = stamp;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;
  cmd->accel = acc;

  accel_pub_->publish(move(cmd));
}

void AccelAngleController::publishAngle(const builtin_interfaces::msg::Time& stamp, const kdl::Euler& angle)
{
  auto cmd = std::make_unique<tobas_command_msgs::Angle>();
  cmd->header.stamp = stamp;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;
  cmd->angle = angle;

  angle_pub_->publish(move(cmd));
}
}  // namespace tobas_rc_teleop

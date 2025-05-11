#include "tobas_rc_teleop/pos_vel_yaw.hpp"

#include <tobas_ros2_tools/time.hpp>

using namespace std;

namespace tobas_rc_teleop
{
PosVelYawController::PosVelYawController()
{
}

bool PosVelYawController::requirePosition()
{
  return true;
}

bool PosVelYawController::requireOrientation()
{
  return false;
}

bool PosVelYawController::requireLinearVelocity()
{
  return true;
}

bool PosVelYawController::requireAngularVelocity()
{
  return false;
}

void PosVelYawController::initialize(tobas::BaseNode* node, tobas::flight_mode_t mode)
{
  node->addDynamicDoubleParam(
    addMode("max_horizontal_velocity", mode), &self::maxHorizontalVelocityCb, this, 6., 0., 10.);
  node->addDynamicDoubleParam(addMode("max_vertical_velocity", mode), &self::maxVerticalVelocityCb, this, 4., 0., 10.);
  node->addDynamicDoubleParam(addMode("max_heading_rate", mode), &self::maxHeadingRateCb, this, M_PI_2, 0., M_PI * 2);
  node->addDynamicIntParam(
    addMode("horizontal_velocity_expo", mode), &self::horizontalVelocityExpoCb, this, 0, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(
    addMode("vertical_velocity_expo", mode), &self::verticalVelocityExpoCb, this, 0, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("heading_expo", mode), &self::headingExpoCb, this, 0, -kExpoScale, kExpoScale);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::PosVelYaw>(tobas::kPosVelYawCmdTopic);
}

void PosVelYawController::reset(const tobas_msgs::Odometry& odom)
{
  t_last_rcin_ = odom.header.stamp;
  tar_pos_W_ = odom.frame.p;
  tar_vel_G_.setZero();
  tar_yaw_ = odom.frame.M.getYaw();
}

void PosVelYawController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom)
{
  // 時刻を更新
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // RC入力を地面座標系から見た速度とヨーレートに変換
  tar_vel_G_.x(expoRemapDead(rcin.pitch, hor_vel_expo_, -max_hor_vel_, max_hor_vel_));
  tar_vel_G_.y(-expoRemapDead(rcin.roll, hor_vel_expo_, -max_hor_vel_, max_hor_vel_));
  tar_vel_G_.z(expoRemapDead(rcin.throttle, ver_vel_expo_, -max_ver_vel_, max_ver_vel_));
  const auto yawrate = expoRemapDead(rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_);

  // 目標速度を地面座標系から世界座標系に変換
  // ヨー角の現在値で変換すると直進指令でも進路が曲がってしまうため，指令値で変換する．
  const auto tar_vel_W = kdl::Rotation::RotZ(tar_yaw_) * tar_vel_G_;

  // 目標速度とヨーレートを積分
  tar_pos_W_ += tar_vel_W * dt;
  tar_yaw_ += yawrate * dt;

  // 目標位置の偏差を制限
  const auto& cur_pos_W = odom.frame.p;
  tar_pos_W_ = tar_pos_W_.clamp(cur_pos_W - kMaxPositionError, cur_pos_W + kMaxPositionError);

  // コマンドを作成
  auto cmd = make_unique<tobas_command_msgs::PosVelYaw>();
  cmd->header = rcin.header;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;
  cmd->pos = tar_pos_W_;
  cmd->vel = tar_vel_W;
  cmd->yaw = tar_yaw_;

  // コマンドを発行
  cmd_pub_->publish(move(cmd));
}

bool PosVelYawController::maxHorizontalVelocityCb(const double& p)
{
  max_hor_vel_ = p;
  return true;
}

bool PosVelYawController::maxVerticalVelocityCb(const double& p)
{
  max_ver_vel_ = p;
  return true;
}

bool PosVelYawController::maxHeadingRateCb(const double& p)
{
  max_head_rate_ = p;
  return true;
}

bool PosVelYawController::horizontalVelocityExpoCb(const long& p)
{
  hor_vel_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool PosVelYawController::verticalVelocityExpoCb(const long& p)
{
  ver_vel_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool PosVelYawController::headingExpoCb(const long& p)
{
  head_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}
}  // namespace tobas_rc_teleop

#include "tobas_rc_teleop/pos_vel_acc_pitch_yaw.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas_rc_teleop
{
PosVelAccPitchYawController::PosVelAccPitchYawController()
{
}

bool PosVelAccPitchYawController::requirePosition()
{
  return true;
}

bool PosVelAccPitchYawController::requireVelocity()
{
  return true;
}

bool PosVelAccPitchYawController::requireAttitude()
{
  return true;
}

bool PosVelAccPitchYawController::requireHeading()
{
  return true;
}

void PosVelAccPitchYawController::initialize(tobas::BaseNode* node, tobas::FlightMode mode)
{
  node->addDynamicDoubleParam(
    addMode("max_horizontal_velocity", mode), &self::maxHorizontalVelocityCb, this, 0.5, 12, 0, 20, " m/s");
  node->addDynamicDoubleParam(
    addMode("max_vertical_velocity", mode), &self::maxVerticalVelocityCb, this, 0.5, 8, 0, 20, " m/s");
  node->addDynamicDoubleParam(addMode("max_pitch", mode), &self::maxPitchCb, this, 10., 9, 1, 18, " deg");
  node->addDynamicDoubleParam(addMode("max_yaw_rate", mode), &self::maxYawRateCb, this, 20., 9, 1, 18, " dps");
  node->addDynamicDoubleParam(
    addMode("max_position_error_down", mode), &self::maxPositionErrorDown, this, 0.5, 4, 0, 20, " m");
  node->addDynamicDoubleParam(
    addMode("horizontal_velocity_expo", mode), &self::horizontalVelocityExpoCb, this, 5., -6, -20, 20);
  node->addDynamicDoubleParam(
    addMode("vertical_velocity_expo", mode), &self::verticalVelocityExpoCb, this, 5., 0, -20, 20);
  node->addDynamicDoubleParam(addMode("pitch_expo", mode), &self::pitchExpoCb, this, 5., 0, -20, 20);
  node->addDynamicDoubleParam(addMode("yaw_expo", mode), &self::yawExpoCb, this, 5., -3, -20, 20);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::PosVelAccPitchYaw>(tobas::topic::kPosVelAccPitchYawCmd);
}

void PosVelAccPitchYawController::reset(const tobas_msgs::Odometry& odom, bool landed)
{
  t_last_rcin_ = odom.header.stamp;

  tar_pos_W_ = odom.frame.p;
  if (landed) {
    tar_pos_W_.z() -= max_ep_down_;
  }

  tar_pitch_ = odom.frame.M.getPitch();
  tar_yaw_ = odom.frame.M.getYaw();
}

void PosVelAccPitchYawController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed)
{
  // 時刻を更新
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // サブモードで並進制御モードと回転制御モードを切り替える
  if (rcin.sub_mode)  // 回転固定で並進制御
  {
    tar_vel_G_.x(expoRemapDead(rcin.pitch, hor_vel_expo_, -max_hor_vel_, max_hor_vel_));
    tar_pitch_ = 0.;
  }
  else  // 並進固定で回転制御
  {
    tar_vel_G_.x(0.);
    tar_pitch_ = expoRemapDead(rcin.pitch, pitch_expo_, -max_pitch_, max_pitch_);
  }

  // Y, Z, Yaw はサブモードに依らない
  tar_vel_G_.y(-expoRemapDead(rcin.roll, hor_vel_expo_, -max_hor_vel_, max_hor_vel_));
  tar_vel_G_.z(expoRemapDead(rcin.throttle, ver_vel_expo_, -max_ver_vel_, max_ver_vel_));
  const auto yawrate = expoRemapDead(rcin.yaw, yaw_expo_, -max_yaw_rate_, max_yaw_rate_);

  // 目標速度を地面座標系から世界座標系に変換
  // ヨー角の現在値で変換すると直進指令でも進路が曲がってしまうため，指令値で変換する．
  const auto tar_vel_W = kdl::Rotation::RotZ(tar_yaw_) * tar_vel_G_;

  // 目標速度とヨーレートを積分
  tar_pos_W_ += tar_vel_W * dt;
  tar_yaw_ += yawrate * dt;

  // 着陸状態ならば水平位置制御は行わない
  const auto& cur_pos_W = odom.frame.p;
  if (landed) {
    tar_pos_W_.x() = cur_pos_W.x();
    tar_pos_W_.y() = cur_pos_W.y();
  }

  // 着陸時に目標高度が下がりすぎるのを防ぐために偏差を制限
  const auto& cur_z = cur_pos_W.z();
  tar_pos_W_.z() = std::max(tar_pos_W_.z(), cur_z - max_ep_down_);

  // コマンドを作成
  auto cmd = std::make_unique<tobas_command_msgs::PosVelAccPitchYaw>();
  cmd->header = rcin.header;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
  cmd->pos = tar_pos_W_;
  cmd->vel = tar_vel_W;
  cmd->acc.setZero();
  cmd->pitch = tar_pitch_;
  cmd->yaw = tar_yaw_;

  // コマンドを発行
  cmd_pub_->publish(std::move(cmd));
}

bool PosVelAccPitchYawController::maxHorizontalVelocityCb(const double& p)
{
  max_hor_vel_ = p;
  return true;
}

bool PosVelAccPitchYawController::maxVerticalVelocityCb(const double& p)
{
  max_ver_vel_ = p;
  return true;
}

bool PosVelAccPitchYawController::maxPitchCb(const double& p)
{
  max_pitch_ = tbs::deg2rad(p);
  return true;
}

bool PosVelAccPitchYawController::maxYawRateCb(const double& p)
{
  max_yaw_rate_ = tbs::deg2rad(p);
  return true;
}

bool PosVelAccPitchYawController::maxPositionErrorDown(const double& p)
{
  max_ep_down_ = p;
  return true;
}

bool PosVelAccPitchYawController::horizontalVelocityExpoCb(const double& p)
{
  hor_vel_expo_ = p / kExpoScale;
  return true;
}

bool PosVelAccPitchYawController::verticalVelocityExpoCb(const double& p)
{
  ver_vel_expo_ = p / kExpoScale;
  return true;
}

bool PosVelAccPitchYawController::pitchExpoCb(const double& p)
{
  pitch_expo_ = p / kExpoScale;
  return true;
}

bool PosVelAccPitchYawController::yawExpoCb(const double& p)
{
  yaw_expo_ = p / kExpoScale;
  return true;
}
}  // namespace tobas_rc_teleop

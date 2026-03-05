#include "tobas_rc_teleop/pos_vel_acc_angle.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas_rc_teleop
{
PosVelAccAngleController::PosVelAccAngleController()
{
}

bool PosVelAccAngleController::requirePosition()
{
  return true;
}

bool PosVelAccAngleController::requireVelocity()
{
  return true;
}

bool PosVelAccAngleController::requireAttitude()
{
  return true;
}

bool PosVelAccAngleController::requireHeading()
{
  return true;
}

void PosVelAccAngleController::initialize(tobas::BaseNode* node, tobas::FlightMode mode)
{
  node->addDynamicDoubleParam(
    addMode("max_horizontal_velocity", mode), &self::maxHorizontalVelocityCb, this, 0.5, 12, 0, 20, " m/s");
  node->addDynamicDoubleParam(
    addMode("max_vertical_velocity", mode), &self::maxVerticalVelocityCb, this, 0.5, 8, 0, 20, " m/s");
  node->addDynamicDoubleParam(addMode("max_attitude", mode), &self::maxAttitudeCb, this, 10., 9, 1, 18, " deg");
  node->addDynamicDoubleParam(addMode("max_heading_rate", mode), &self::maxHeadingRateCb, this, 20., 9, 1, 18, " dps");
  node->addDynamicDoubleParam(
    addMode("max_position_error_down", mode), &self::maxPositionErrorDown, this, 0.5, 4, 0, 20, " m");
  node->addDynamicDoubleParam(
    addMode("horizontal_velocity_expo", mode), &self::horizontalVelocityExpoCb, this, 5., -6, -20, 20);
  node->addDynamicDoubleParam(
    addMode("vertical_velocity_expo", mode), &self::verticalVelocityExpoCb, this, 5., 0, -20, 20);
  node->addDynamicDoubleParam(addMode("attitude_expo", mode), &self::attitudeExpoCb, this, 5., 0, -20, 20);
  node->addDynamicDoubleParam(addMode("heading_expo", mode), &self::headingExpoCb, this, 5., -3, -20, 20);

  pos_vel_acc_pub_ = node->createPublisher<tobas_command_msgs::PosVelAcc>(tobas::topic::kPosVelAccCmd);
  angle_pub_ = node->createPublisher<tobas_command_msgs::Angle>(tobas::topic::kAngleCmd);
}

void PosVelAccAngleController::reset(const tobas_msgs::Odometry& odom, bool landed)
{
  t_last_rcin_ = odom.header.stamp;

  tar_pos_W_ = odom.frame.p;
  if (landed) {
    tar_pos_W_.z() -= max_ep_down_;
  }

  odom.frame.M.getRPY(tar_angle_.roll, tar_angle_.pitch, tar_angle_.yaw);
}

void PosVelAccAngleController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed)
{
  // 時刻を更新
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // サブモードで並進制御モードと回転制御モードを切り替える
  if (rcin.sub_mode)  // 回転固定で並進制御
  {
    // RC入力から目標水平速度を計算
    tar_vel_G_.x(expoRemapDead(rcin.pitch, hor_vel_expo_, -max_hor_vel_, max_hor_vel_));
    tar_vel_G_.y(-expoRemapDead(rcin.roll, hor_vel_expo_, -max_hor_vel_, max_hor_vel_));

    // 目標姿勢角はゼロ
    tar_angle_.roll = 0.;
    tar_angle_.pitch = 0.;
  }
  else  // 並進固定で回転制御
  {
    // RC入力から目標姿勢を計算
    tar_angle_.roll = expoRemapDead(rcin.roll, atti_expo_, -max_attitude_, max_attitude_);
    tar_angle_.pitch = expoRemapDead(rcin.pitch, atti_expo_, -max_attitude_, max_attitude_);

    // 目標水平速度はゼロ
    tar_vel_G_.x(0.);
    tar_vel_G_.y(0.);
  }

  // RC入力から鉛直速度とヨーレートを計算
  tar_vel_G_.z(expoRemapDead(rcin.throttle, ver_vel_expo_, -max_ver_vel_, max_ver_vel_));
  const auto yawrate = expoRemapDead(rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_);

  // 目標速度を地面座標系から世界座標系に変換
  // ヨー角の現在値で変換すると直進指令でも進路が曲がってしまうため，指令値で変換する．
  const auto tar_vel_W = kdl::Rotation::RotZ(tar_angle_.yaw) * tar_vel_G_;

  // 目標速度とヨーレートを積分
  tar_pos_W_ += tar_vel_W * dt;
  tar_angle_.yaw += yawrate * dt;

  // 着陸状態ならば水平位置制御は行わない
  const auto& cur_pos_W = odom.frame.p;
  if (landed) {
    tar_pos_W_.x() = cur_pos_W.x();
    tar_pos_W_.y() = cur_pos_W.y();
  }

  // 着陸時に目標高度が下がりすぎるのを防ぐために偏差を制限
  const auto& cur_z = cur_pos_W.z();
  tar_pos_W_.z() = std::max(tar_pos_W_.z(), cur_z - max_ep_down_);

  // コマンドを発行
  publishPosVelAcc(rcin.header.stamp, tar_pos_W_, tar_vel_W, kdl::Vector::Zero());
  publishAngle(rcin.header.stamp, tar_angle_);
}

void PosVelAccAngleController::publishPosVelAcc(
  const builtin_interfaces::msg::Time& stamp,
  const kdl::Vector& pos,
  const kdl::Vector& vel,
  const kdl::Vector& acc)
{
  auto cmd = std::make_unique<tobas_command_msgs::PosVelAcc>();
  cmd->header.stamp = stamp;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
  cmd->pos = pos;
  cmd->vel = vel;
  cmd->acc = acc;

  pos_vel_acc_pub_->publish(std::move(cmd));
}

void PosVelAccAngleController::publishAngle(const builtin_interfaces::msg::Time& stamp, const kdl::Euler& angle)
{
  auto cmd = std::make_unique<tobas_command_msgs::Angle>();
  cmd->header.stamp = stamp;
  cmd->priority.data = tobas_command_msgs::msg::Priority::MANUAL;
  cmd->angle = angle;

  angle_pub_->publish(std::move(cmd));
}

bool PosVelAccAngleController::maxHorizontalVelocityCb(const double& p)
{
  max_hor_vel_ = p;
  return true;
}

bool PosVelAccAngleController::maxVerticalVelocityCb(const double& p)
{
  max_ver_vel_ = p;
  return true;
}

bool PosVelAccAngleController::maxAttitudeCb(const double& p)
{
  max_attitude_ = tbs::deg2rad(p);
  return true;
}

bool PosVelAccAngleController::maxHeadingRateCb(const double& p)
{
  max_head_rate_ = tbs::deg2rad(p);
  return true;
}

bool PosVelAccAngleController::maxPositionErrorDown(const double& p)
{
  max_ep_down_ = p;
  return true;
}

bool PosVelAccAngleController::horizontalVelocityExpoCb(const double& p)
{
  hor_vel_expo_ = p / kExpoScale;
  return true;
}

bool PosVelAccAngleController::verticalVelocityExpoCb(const double& p)
{
  ver_vel_expo_ = p / kExpoScale;
  return true;
}

bool PosVelAccAngleController::attitudeExpoCb(const double& p)
{
  atti_expo_ = p / kExpoScale;
  return true;
}

bool PosVelAccAngleController::headingExpoCb(const double& p)
{
  head_expo_ = p / kExpoScale;
  return true;
}
}  // namespace tobas_rc_teleop

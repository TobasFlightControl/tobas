#include <tobas_ros2_tools/time.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_rc_teleop/pos_vel_angle.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;

namespace tobas_rc_teleop
{
PosVelAngleController::PosVelAngleController()
{
}

bool PosVelAngleController::requirePosition()
{
  return true;
}

bool PosVelAngleController::requireOrientation()
{
  return true;
}

bool PosVelAngleController::requireLinearVelocity()
{
  return true;
}

bool PosVelAngleController::requireAngularVelocity()
{
  return true;
}

void PosVelAngleController::initialize(tobas::BaseNode* node)
{
  getStaticRosParams(node);

  pos_vel_pub_ = node->createPublisher<tobas_command_msgs::PosVel>(tobas::kPosVelCmdTopic);
  angle_pub_ = node->createPublisher<tobas_command_msgs::Angle>(tobas::kAngleCmdTopic);
}

void PosVelAngleController::reset(const tobas_msgs::Odometry& odom)
{
  is_up_commanded_ = false;
  t_last_rcin_ = odom.header.stamp;
  tar_vel_G_.setZero();
  tar_pos_W_ = odom.frame.p;
  odom.frame.M.getRPY(tar_angle_.roll, tar_angle_.pitch, tar_angle_.yaw);
}

void PosVelAngleController::update(const tobas_msgs::msg::RCInput& rcin, const tobas_msgs::Odometry& odom)
{
  // 時刻を更新
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // GPSwの状態によって水平速度制御モードと姿勢制御モードを切り替える
  if (rcin.gpsw)  // 姿勢固定で位置制御
  {
    // RC入力から目標水平速を計算
    tar_vel_G_.x(remapDead(rcin.pitch, -max_hor_vel_, max_hor_vel_));
    tar_vel_G_.y(-remapDead(rcin.roll, -max_hor_vel_, max_hor_vel_));

    // 目標姿勢角はゼロ
    tar_angle_.roll = 0.;
    tar_angle_.pitch = 0.;
  }
  else  // 位置固定で姿勢制御
  {
    // RC入力から目標姿勢を計算
    tar_angle_.roll = remapDead(rcin.roll, -max_attitude_, max_attitude_);
    tar_angle_.pitch = remapDead(rcin.pitch, -max_attitude_, max_attitude_);

    // 目標水平速度はゼロ
    tar_vel_G_.x(0.);
    tar_vel_G_.y(0.);
  }

  // RC入力から鉛直速度とヨーレートを計算
  tar_vel_G_.z(remapDead(rcin.throttle, -max_ver_vel_, max_ver_vel_));
  const auto yawrate = remapDead(rcin.yaw, -max_heading_rate_, max_heading_rate_);

  // 目標速度を地面座標系から世界座標系に変換
  // ヨー角の現在値で変換すると直進指令でも進路が曲がってしまうため，指令値で変換する．
  const auto tar_vel_W = kdl::Rotation::RotZ(tar_angle_.yaw) * tar_vel_G_;

  // 目標速度とヨーレートを積分
  tar_pos_W_ += tar_vel_W * dt;
  tar_angle_.yaw += yawrate * dt;

  // 目標位置の偏差を制限
  const auto& cur_pos_W = odom.frame.p;
  tar_pos_W_ = tar_pos_W_.clamp(cur_pos_W - kMaxPositionError, cur_pos_W + kMaxPositionError);

  // 上昇コマンドが入力されるまでは位置とヨーの制御は行わない
  if (!is_up_commanded_)
  {
    tar_pos_W_ = cur_pos_W;
    tar_angle_.yaw = odom.frame.M.getYaw();
    is_up_commanded_ = tar_vel_W.z() > 0;
  }

  // コマンドを発行
  publishPosVel(rcin.header.stamp, tar_pos_W_, tar_vel_W);
  publishAngle(rcin.header.stamp, tar_angle_);
}

void PosVelAngleController::getStaticRosParams(tobas::BaseNode* node)
{
  max_hor_vel_ = node->getDoubleParam("max_horizontal_velocity", kDefaultMaxHorVel);
  if (max_hor_vel_ < 0)
  {
    node->error("Maximum horizontal velocity must be positive.");
    max_hor_vel_ = kDefaultMaxHorVel;
  }

  max_ver_vel_ = node->getDoubleParam("max_vertical_velocity", kDefaultMaxVerVel);
  if (max_ver_vel_ < 0)
  {
    node->error("Maximum vertical velocity must be positive.");
    max_ver_vel_ = kDefaultMaxVerVel;
  }

  max_attitude_ = node->getDoubleParam("max_attitude", kDefaultMaxAttitude);
  if (max_attitude_ < 0)
  {
    node->error("Maximum attitude angle must be positive.");
    max_attitude_ = kDefaultMaxAttitude;
  }

  max_heading_rate_ = node->getDoubleParam("max_heading_rate", kDefaultMaxHeadingRate);
  if (max_heading_rate_ < 0)
  {
    node->error("Maximum heading rate must be positive.");
    max_heading_rate_ = kDefaultMaxHeadingRate;
  }
}

void PosVelAngleController::publishPosVel(
  const builtin_interfaces::msg::Time& stamp,
  const kdl::Vector& pos,
  const kdl::Vector& vel)
{
  auto cmd = std::make_unique<tobas_command_msgs::PosVel>();
  cmd->header.stamp = stamp;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;
  cmd->pos = pos;
  cmd->vel = vel;

  pos_vel_pub_->publish(move(cmd));
}

void PosVelAngleController::publishAngle(const builtin_interfaces::msg::Time& stamp, const kdl::Euler& angle)
{
  auto cmd = std::make_unique<tobas_command_msgs::Angle>();
  cmd->header.stamp = stamp;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;
  cmd->angle = angle;

  angle_pub_->publish(move(cmd));
}
}  // namespace tobas_rc_teleop

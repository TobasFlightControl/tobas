#include <tobas_kdl/euler.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_rc_teleop/pos_vel_acc_yaw.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;

namespace tobas_rc_teleop
{
PosVelAccYawController::PosVelAccYawController()
{
}

bool PosVelAccYawController::requirePosition()
{
  return true;
}

bool PosVelAccYawController::requireOrientation()
{
  return false;
}

bool PosVelAccYawController::requireLinearVelocity()
{
  return true;
}

bool PosVelAccYawController::requireAngularVelocity()
{
  return false;
}

void PosVelAccYawController::initialize(tobas::BaseNode* node)
{
  getStaticRosParams(node);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::PosVelAccYaw>(tobas::kPosVelAccYawCmdTopic);
}

void PosVelAccYawController::reset(const tobas_msgs::Odometry& odom)
{
  is_up_commanded_ = false;
  t_last_rcin_ = odom.header.stamp;
  tar_pos_W_ = odom.frame.p;
  tar_vel_F_.setZero();
  tar_yaw_ = kdl::Euler(odom.frame.M).yaw;
}

void PosVelAccYawController::update(const tobas_msgs::msg::RCInput& rcin, const tobas_msgs::Odometry& odom)
{
  // 時刻を更新
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // RC入力を速度とヨーレートに変換
  tar_vel_F_.x(remapDead(rcin.pitch, -max_hor_vel_, max_hor_vel_));
  tar_vel_F_.y(-remapDead(rcin.roll, -max_hor_vel_, max_hor_vel_));
  tar_vel_F_.z(remapDead(rcin.throttle, -max_ver_vel_, max_ver_vel_));
  const auto yawrate = remapDead(rcin.yaw, -max_heading_rate_, max_heading_rate_);

  // 目標速度を世界座標系に変換
  // ヨー角の現在値で変換すると直進指令でも進路が曲がってしまうため，指令値で変換する．
  const auto tar_vel_W = kdl::Rotation::RotZ(tar_yaw_) * tar_vel_F_;

  // 目標速度とヨーレートを積分
  tar_pos_W_ += tar_vel_W * dt;
  tar_yaw_ += yawrate * dt;

  // 目標位置の偏差を制限
  const auto& cur_pos_W = odom.frame.p;
  const auto cur_yaw = kdl::Euler(odom.frame.M).yaw;
  tar_pos_W_ = tar_pos_W_.clamp(cur_pos_W - kMaxPositionError, cur_pos_W + kMaxPositionError);

  // 上昇コマンドが入力されるまでは位置とヨーの制御は行わない
  if (!is_up_commanded_)
  {
    tar_pos_W_ = cur_pos_W;
    tar_yaw_ = cur_yaw;
    is_up_commanded_ = tar_vel_F_.z() > 0;
  }

  // コマンドを作成
  auto cmd = std::make_unique<tobas_command_msgs::PosVelAccYaw>();
  cmd->header = rcin.header;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;
  cmd->frame_id.data = tobas_command_msgs::msg::FrameId::WORLD;
  cmd->pos = tar_pos_W_;
  cmd->vel = tar_vel_W;
  cmd->acc.setZero();
  cmd->yaw = tar_yaw_;

  // コマンドを発行
  cmd_pub_->publish(move(cmd));
}

void PosVelAccYawController::getStaticRosParams(tobas::BaseNode* node)
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

  max_heading_rate_ = node->getDoubleParam("max_heading_rate", kDefaultMaxHeadingRate);
  if (max_heading_rate_ < 0)
  {
    node->error("Maximum heading rate must be positive.");
    max_heading_rate_ = kDefaultMaxHeadingRate;
  }
}
}  // namespace tobas_rc_teleop

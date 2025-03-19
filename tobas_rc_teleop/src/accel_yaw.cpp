#include <tobas_ros2_tools/time.hpp>

#include "../include/tobas_rc_teleop/accel_yaw.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;

namespace tobas_rc_teleop
{
AccelYawController::AccelYawController()
{
}

bool AccelYawController::requirePosition()
{
  return false;
}

bool AccelYawController::requireOrientation()
{
  return true;
}

bool AccelYawController::requireLinearVelocity()
{
  return false;
}

bool AccelYawController::requireAngularVelocity()
{
  return false;
}

void AccelYawController::initialize(tobas::BaseNode* node)
{
  getStaticRosParams(node);

  cmd_pub_ = node->createPublisher<tobas_command_msgs::AccelYaw>(tobas::kAccelYawCmdTopic);
}

void AccelYawController::reset(const tobas_msgs::Odometry& odom)
{
  t_last_rcin_ = odom.header.stamp;
  tar_yaw_ = odom.frame.M.getYaw();
}

void AccelYawController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry&)
{
  // 時刻を更新
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // RC入力を地面座標系から見た加速度とヨーレートに変換
  tar_acc_G_.x(remap(rcin.pitch, -max_hor_acc_, max_hor_acc_));
  tar_acc_G_.y(-remap(rcin.roll, -max_hor_acc_, max_hor_acc_));
  tar_acc_G_.z(remap(rcin.throttle, -max_ver_acc_, max_ver_acc_));
  const auto yawrate = remapDead(rcin.yaw, -max_head_rate_, max_head_rate_);

  // ヨーレートを積分
  tar_yaw_ += yawrate * dt;

  // コマンドを作成
  auto cmd = std::make_unique<tobas_command_msgs::AccelYaw>();
  cmd->header = rcin.header;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;
  cmd->accel = kdl::Rotation::RotZ(tar_yaw_) * tar_acc_G_;  // 地面座標系から世界座標系に変換
  cmd->yaw = tar_yaw_;

  // コマンドを発行
  cmd_pub_->publish(move(cmd));
}

void AccelYawController::getStaticRosParams(tobas::BaseNode* node)
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

  max_head_rate_ = node->getDoubleParam("max_heading_rate", kDefaultMaxHeadingRate);
  if (max_head_rate_ < 0)
  {
    node->error("Maximum heading rate must be positive.");
    max_head_rate_ = kDefaultMaxHeadingRate;
  }
}
}  // namespace tobas_rc_teleop

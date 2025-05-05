#include "../include/tobas_rc_teleop/accel_yaw.hpp"

#include <tobas_ros2_tools/time.hpp>

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

void AccelYawController::initialize(tobas::BaseNode* node, tobas::flight_mode_t mode)
{
  node->addDynamicDoubleParam(addMode("max_horizontal_accel", mode), &self::maxHorizontalAccelCb, this, 5., 0., 10.);
  node->addDynamicDoubleParam(addMode("max_vertical_accel", mode), &self::maxVerticalAccelCb, this, 4., 0., 10.);
  node->addDynamicDoubleParam(addMode("max_heading_rate", mode), &self::maxHeadingRateCb, this, M_PI_2, 0., M_PI * 2);
  node->addDynamicIntParam(
    addMode("horizontal_accel_expo", mode), &self::horizontalAccelExpoCb, this, 0, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(
    addMode("vertical_accel_expo", mode), &self::verticalAccelExpoCb, this, 0, -kExpoScale, kExpoScale);
  node->addDynamicIntParam(addMode("heading_expo", mode), &self::headingExpoCb, this, 0, -kExpoScale, kExpoScale);

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
  tar_acc_G_.x(expoRemap(rcin.pitch, hor_acc_expo_, -max_hor_acc_, max_hor_acc_));
  tar_acc_G_.y(-expoRemap(rcin.roll, hor_acc_expo_, -max_hor_acc_, max_hor_acc_));
  tar_acc_G_.z(expoRemap(rcin.throttle, ver_acc_expo_, -max_ver_acc_, max_ver_acc_));
  const auto yawrate = expoRemapDead(rcin.yaw, head_expo_, -max_head_rate_, max_head_rate_);

  // ヨーレートを積分
  tar_yaw_ += yawrate * dt;

  // コマンドを作成
  auto cmd = make_unique<tobas_command_msgs::AccelYaw>();
  cmd->header = rcin.header;
  cmd->level.data = tobas_command_msgs::msg::CommandLevel::MANUAL;
  cmd->accel = kdl::Rotation::RotZ(tar_yaw_) * tar_acc_G_;  // 地面座標系から世界座標系に変換
  cmd->yaw = tar_yaw_;

  // コマンドを発行
  cmd_pub_->publish(move(cmd));
}

bool AccelYawController::maxHorizontalAccelCb(const double& p)
{
  max_hor_acc_ = p;
  return true;
}

bool AccelYawController::maxVerticalAccelCb(const double& p)
{
  max_ver_acc_ = p;
  return true;
}

bool AccelYawController::maxHeadingRateCb(const double& p)
{
  max_head_rate_ = p;
  return true;
}

bool AccelYawController::horizontalAccelExpoCb(const long& p)
{
  hor_acc_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool AccelYawController::verticalAccelExpoCb(const long& p)
{
  ver_acc_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}

bool AccelYawController::headingExpoCb(const long& p)
{
  head_expo_ = static_cast<double>(p) / kExpoScale;
  return true;
}
}  // namespace tobas_rc_teleop

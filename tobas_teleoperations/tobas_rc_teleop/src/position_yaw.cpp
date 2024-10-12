#include <tobas_kdl/euler.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_rc_teleop/position_yaw.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;

namespace tobas_rc_teleop
{
PositionYawController::PositionYawController()
{
}

void PositionYawController::initialize(tobas::BaseNode* node)
{
  getStaticRosParams(node);
  pos_yaw_.level.data = tobas_msgs::msg::CommandLevel::MANUAL;
  pos_yaw_pub_ = node->createPublisher<tobas_msgs::PositionYaw>(tobas::kPositionYawCmdTopic);
}

void PositionYawController::reset(const tobas_msgs::Odometry& odom)
{
  pos_yaw_.pos = odom.frame.p;
  pos_yaw_.yaw = kdl::Euler(odom.frame.M).yaw;
  t_last_rcin_ = odom.header.stamp;
}

void PositionYawController::update(const tobas_msgs::msg::RCInput& rcin, const tobas_msgs::Odometry& odom)
{
  // 時刻を更新
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // 位置とヨー角の変化率を計算
  vel_.x(remapDead(rcin.pitch, -max_hor_vel_, max_hor_vel_));
  vel_.y(-remapDead(rcin.roll, -max_hor_vel_, max_hor_vel_));
  vel_.z(remapDead(rcin.throttle, -max_ver_vel_, max_ver_vel_));
  const auto yawrate = remapDead(rcin.yaw, -max_yawrate_, max_yawrate_);

  // 一度でも上昇コマンドが入力されたら位置制御を行う
  if (is_up_commanded_)
  {
    // 速度とヨーレートを積分
    pos_yaw_.pos += vel_ * dt;
    pos_yaw_.yaw += yawrate * dt;
  }
  else
  {
    // 上昇コマンドが入力されるまでは位置とヨーの制御は行わない
    pos_yaw_.pos = odom.frame.p;
    pos_yaw_.yaw = kdl::Euler(odom.frame.M).yaw;

    // 上昇コマンドが入力されたかどうかをチェック
    is_up_commanded_ = vel_.z() > 0;
  }

  // コマンドを発行
  auto pos_yaw_ptr = std::make_unique<tobas_msgs::PositionYaw>(pos_yaw_);
  pos_yaw_pub_->publish(move(pos_yaw_ptr));
}

void PositionYawController::getStaticRosParams(tobas::BaseNode* node)
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

  max_yawrate_ = node->getDoubleParam("max_yawrate", kDefaultMaxYawrate);
  if (max_yawrate_ < 0)
  {
    node->error("Maximum yawrate must be positive.");
    max_yawrate_ = kDefaultMaxYawrate;
  }
}
}  // namespace tobas_rc_teleop

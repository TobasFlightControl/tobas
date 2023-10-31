#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/PosVelAccYaw.h>

#include "../include/tobas_rc_teleop/pos_vel_acc_yaw.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;
using namespace Eigen;
using namespace dh_std;

namespace tobas_rc_teleop
{
PosVelAccYawController::PosVelAccYawController() : super()
{
}

void PosVelAccYawController::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  getRosParams(pnh);
  registerPublishers(nh);
}

void PosVelAccYawController::reset(const tobas_msgs::PoseTwist& pt)
{
  t_last_rcin_ = ros::Time::now();
  vel_filter_.initialize(delay_time_const_, Vector3d::Zero());
  tar_vel_.setZero();
  tar_pos_ = pt.pose.pos.data;
  tar_yaw_ = pt.pose.euler.yaw;
}

void PosVelAccYawController::update(
  const tobas_msgs::RCInput& rcin,
  const dh_std::Range<double>& dead_zone)
{
  // 時刻を更新
  const ros::Time cur_time = ros::Time::now();
  const auto dt = (cur_time - t_last_rcin_).toSec();
  t_last_rcin_ = cur_time;

  // RC入力から速度を計算
  tar_vel_.x() =
    dead_zone.inRange(rcin.pitch) ? 0. : remap(rcin.pitch, -1., 1., -max_hor_vel_, max_hor_vel_);
  tar_vel_.y() =
    dead_zone.inRange(rcin.roll) ? 0. : -remap(rcin.roll, -1., 1., -max_hor_vel_, max_hor_vel_);
  tar_vel_.z() = remap(rcin.thrust, 0., 1., -max_ver_vel_, max_ver_vel_);

  // 速度をフィルタリング
  vel_filter_.update(tar_vel_, dt);
  const Vector3d& tar_vel_filtered = vel_filter_.getState();

  // 位置を更新
  tar_pos_ += tar_vel_filtered * dt;

  // ヨー角を更新
  const auto yawrate =
    dead_zone.inRange(rcin.yaw) ? 0. : remap(rcin.yaw, -1., 1., -max_yawrate_, max_yawrate_);
  tar_yaw_ += yawrate * dt;

  // コマンドを作成
  const auto cmd = boost::make_shared<tobas_msgs::PosVelAccYaw>();
  cmd->level.data = tobas_msgs::CommandLevel::MANUAL;
  cmd->vel_frame.data = tobas_msgs::FrameId::GLOBAL;
  cmd->acc_frame.data = tobas_msgs::FrameId::GLOBAL;
  cmd->vel.data = tar_vel_filtered;
  cmd->pos.data = tar_pos_;
  cmd->yaw = tar_yaw_;

  // コマンドを発行
  cmd_pub_.publish(cmd);
}

void PosVelAccYawController::getRosParams(ros::NodeHandle& pnh)
{
  dh_ros::getParam(
    pnh, "velocity_yaw/max_horizontal_velocity", max_hor_vel_, kDefaultMaxHorizontalVelocity,
    dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "velocity_yaw/max_vertical_velocity", max_ver_vel_, kDefaultMaxVerticalVelocity,
    dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "velocity_yaw/max_yawrate", max_yawrate_, kDefaultMaxYawrate, dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "velocity_yaw/delay_time_const", delay_time_const_, kDefaultDelayTimeConst,
    dh_ros::NON_NEGATIVE);
}

void PosVelAccYawController::registerPublishers(ros::NodeHandle& nh)
{
  cmd_pub_ = nh.advertise<tobas_msgs::PosVelAccYaw>(tobas::kPosVelAccYawCmdTopic, 1);
}
}  // namespace tobas_rc_teleop

#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/PosVelAccYaw.h>

#include "../include/tobas_rc_teleop/pos_vel_acc_yaw.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;
using namespace KDL;
using namespace dh_std;

namespace tobas_rc_teleop
{
void PosVelAccYawController::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  getRosParams(pnh);

  max_pos_err_.x(max_hor_pos_err_);
  max_pos_err_.y(max_hor_pos_err_);
  max_pos_err_.z(max_ver_pos_err_);

  registerPublishers(nh);
}

void PosVelAccYawController::reset(const tobas_msgs::PoseTwist& pt)
{
  t_last_rcin_ = ros::Time::now();
  vel_filter_.initialize(delay_time_const_, Vector::Zero());
  tar_pos_ = pt.pose.pos;
  tar_vel_.setZero();
  tar_yaw_ = pt.pose.euler.yaw;
}

void PosVelAccYawController::update(
  const tobas_msgs::RCInput& rcin,
  const tobas_msgs::PoseTwist& pt,
  const double&,
  const Range<double>& dead_zone)
{
  // 時刻を更新
  const ros::Time cur_time = ros::Time::now();
  const auto dt = (cur_time - t_last_rcin_).toSec();
  t_last_rcin_ = cur_time;

  // RC入力を速度とヨーレートに変換
  tar_vel_.x() =
    dead_zone.inRange(rcin.pitch) ? 0 : remap(rcin.pitch, -1., 1., -max_hor_vel_, max_hor_vel_);
  tar_vel_.y() =
    dead_zone.inRange(rcin.roll) ? 0 : -remap(rcin.roll, -1., 1., -max_hor_vel_, max_hor_vel_);
  tar_vel_.z() = remap(rcin.thrust, 0., 1., -max_ver_vel_, max_ver_vel_);
  const auto yawrate =
    dead_zone.inRange(rcin.yaw) ? 0 : remap(rcin.yaw, -1., 1., -max_yawrate_, max_yawrate_);

  // 目標速度をフィルタリング
  vel_filter_.update(tar_vel_, dt);
  const auto& tar_vel_filtered = vel_filter_.getState();

  // 速度とヨーレートを積分
  tar_pos_ += tar_vel_filtered * dt;
  tar_yaw_ += yawrate * dt;

  // 誤差を制限
  const auto& cur_pos = pt.pose.pos;
  const auto& cur_yaw = pt.pose.euler.yaw;
  tar_pos_ = tar_pos_.clamp(cur_pos - max_pos_err_, cur_pos + max_pos_err_);
  tar_yaw_ = clamp(tar_yaw_, cur_yaw - max_yaw_err_, cur_yaw + max_yaw_err_);

  // コマンドを作成
  const auto cmd = boost::make_shared<tobas_msgs::PosVelAccYaw>();
  cmd->level.data = tobas_msgs::CommandLevel::MANUAL;
  cmd->vel_frame.data = tobas_msgs::FrameId::GLOBAL;
  cmd->acc_frame.data = tobas_msgs::FrameId::GLOBAL;
  cmd->pos = tar_pos_;
  cmd->vel = tar_vel_filtered;
  cmd->acc.setZero();
  cmd->yaw = tar_yaw_;

  // コマンドを発行
  cmd_pub_.publish(cmd);
}

void PosVelAccYawController::getRosParams(ros::NodeHandle& pnh)
{
  dh_ros::getParam(
    pnh, "pos_vel_acc_yaw/max_horizontal_position_error", max_hor_pos_err_, kDefaultMaxHorPosErr,
    dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "pos_vel_acc_yaw/max_vertical_position_error", max_ver_pos_err_, kDefaultMaxVerPosErr,
    dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "pos_vel_acc_yaw/max_horizontal_velocity", max_hor_vel_, kDefaultMaxHorVel,
    dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "pos_vel_acc_yaw/max_vertical_velocity", max_ver_vel_, kDefaultMaxVerVel,
    dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "pos_vel_acc_yaw/max_yawrate", max_yawrate_, kDefaultMaxYawrate, dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "pos_vel_acc_yaw/max_yaw_error", max_yaw_err_, kDefaultMaxYawErr, dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "pos_vel_acc_yaw/delay_time_const", delay_time_const_, kDefaultDelayTimeConst,
    dh_ros::NON_NEGATIVE);
}

void PosVelAccYawController::registerPublishers(ros::NodeHandle& nh)
{
  cmd_pub_ = nh.advertise<tobas_msgs::PosVelAccYaw>(tobas::kPosVelAccYawCmdTopic, 1);
}
}  // namespace tobas_rc_teleop

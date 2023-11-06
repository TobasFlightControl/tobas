#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/PoseTwistAccelCommand.h>

#include "../include/tobas_rc_teleop/pose_twist_accel.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;
using namespace KDL;
using namespace dh_std;

namespace tobas_rc_teleop
{
PoseTwistAccelController::PoseTwistAccelController() : super()
{
}

void PoseTwistAccelController::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  getRosParams(pnh);

  max_pos_err_.x(max_hor_pos_err_);
  max_pos_err_.y(max_hor_pos_err_);
  max_pos_err_.z(max_ver_pos_err_);

  registerPublishers(nh);
}

void PoseTwistAccelController::reset(const tobas_msgs::PoseTwist& pt)
{
  t_last_rcin_ = ros::Time::now();
  setToZero(tar_vel_);
  tar_pos_ = pt.pose.pos;
  tar_rpy_.yaw = pt.pose.euler.yaw;
}

void PoseTwistAccelController::update(
  const tobas_msgs::RCInput& rcin,
  const tobas_msgs::PoseTwist& pt,
  const dh_std::Range<double>& dead_zone)
{
  // 時刻を更新
  const ros::Time cur_time = ros::Time::now();
  const auto dt = (cur_time - t_last_rcin_).toSec();
  t_last_rcin_ = cur_time;

  // Aliases
  const auto& cur_pos = pt.pose.pos;
  const auto& cur_yaw = pt.pose.euler.yaw;

  // GPSw-1の状態によって水平速度制御モードと姿勢制御モードを切り替える
  if (rcin.gpsw1)  // 姿勢固定で位置制御
  {
    // RC入力から目標水平速を計算
    tar_vel_.x() =
      dead_zone.inRange(rcin.pitch) ? 0 : remap(rcin.pitch, -1., 1., -max_hor_vel_, max_hor_vel_);
    tar_vel_.y() =
      dead_zone.inRange(rcin.roll) ? 0 : -remap(rcin.roll, -1., 1., -max_hor_vel_, max_hor_vel_);

    // 目標姿勢角はゼロ
    tar_rpy_.roll = 0;
    tar_rpy_.pitch = 0;
  }
  if (rcin.gpsw1)  // 位置固定で姿勢制御
  {
    // RC入力から目標姿勢を計算
    tar_rpy_.roll =
      dead_zone.inRange(rcin.roll) ? 0 : -remap(rcin.roll, -1., 1., -max_attitude_, max_attitude_);
    tar_rpy_.pitch =
      dead_zone.inRange(rcin.pitch) ? 0 : remap(rcin.pitch, -1., 1., -max_attitude_, max_attitude_);

    // 目標水平速度はゼロ
    tar_vel_.x() = 0;
    tar_vel_.y() = 0;
  }

  // 目標鉛直速度
  tar_vel_.z() = remap(rcin.thrust, 0., 1., -max_ver_vel_, max_ver_vel_);

  // 目標速度を積分して目標位置を計算
  // 離陸前に誤差が過大になるのを防ぐため，現在の位置との誤差を制限する
  tar_pos_ += tar_vel_ * dt;
  tar_pos_ = tar_pos_.clamp(cur_pos - max_pos_err_, cur_pos + max_pos_err_);

  // 目標ヨー角を更新
  const auto yawrate =
    dead_zone.inRange(rcin.yaw) ? 0 : remap(rcin.yaw, -1., 1., -max_yawrate_, max_yawrate_);
  tar_rpy_.yaw = clamp(tar_rpy_.yaw + yawrate * dt, cur_yaw - max_yaw_err_, cur_yaw + max_yaw_err_);

  // コマンドを作成
  const auto cmd = boost::make_shared<tobas_msgs::PoseTwistAccelCommand>();
  cmd->level.data = tobas_msgs::CommandLevel::MANUAL;
  cmd->vel = tar_vel_;
  cmd->pos = tar_pos_;
  cmd->rpy = tar_rpy_;

  // コマンドを発行
  cmd_pub_.publish(cmd);
}

void PoseTwistAccelController::getRosParams(ros::NodeHandle& pnh)
{
  dh_ros::getParam(
    pnh, "pose_twist_accel/max_horizontal_position_error", max_hor_pos_err_, kDefaultMaxHorPosErr,
    dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "pose_twist_accel/max_vertical_position_error", max_ver_pos_err_, kDefaultMaxVerPosErr,
    dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "pose_twist_accel/max_horizontal_velocity", max_hor_vel_, kDefaultMaxHorVel,
    dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "pose_twist_accel/max_vertical_velocity", max_ver_vel_, kDefaultMaxVerVel,
    dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "pose_twist_accel/max_attitude", max_attitude_, kDefaultMaxAttitude, dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "pose_twist_accel/max_yawrate", max_yawrate_, kDefaultMaxYawrate, dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "pose_twist_accel/max_yaw_error", max_yaw_err_, kDefaultMaxYawErr, dh_ros::POSITIVE);
}

void PoseTwistAccelController::registerPublishers(ros::NodeHandle& nh)
{
  cmd_pub_ = nh.advertise<tobas_msgs::PoseTwistAccelCommand>(tobas::kPoseTwistAccelCmdTopic, 1);
}
}  // namespace tobas_rc_teleop

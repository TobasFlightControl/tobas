#include <tobas_std_tools/math.hpp>
#include <tobas_kdl/euler.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/PoseTwistAccelCommand.h>

#include "../include/tobas_rc_teleop/pose_twist_accel.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;
using namespace KDL;
using namespace tobas_std;

namespace tobas_rc_teleop
{
PoseTwistAccelController::PoseTwistAccelController(const tobas::Drone& drone) : super(drone)
{
}

void PoseTwistAccelController::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  getRosParams(pnh);
  registerPublishers(nh);
}

void PoseTwistAccelController::reset(const tobas_msgs::Odometry& odom)
{
  t_last_rcin_ = ros::Time::now();
  setToZero(tar_vel_);
  tar_pos_ = odom.frame.p;
  odom.frame.M.getRPY(tar_rpy_.roll, tar_rpy_.pitch, tar_rpy_.yaw);
}

void PoseTwistAccelController::update(
  const tobas_msgs::RCInput& rcin,
  const tobas_msgs::Odometry& odom,
  const double&,
  const Range<double>& dead_zone)
{
  // 時刻を更新
  const auto cur_time = ros::Time::now();
  const auto dt = (cur_time - t_last_rcin_).toSec();
  t_last_rcin_ = cur_time;

  // GPSwの状態によって水平速度制御モードと姿勢制御モードを切り替える
  if (rcin.gpsw)  // 姿勢固定で位置制御
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
  else  // 位置固定で姿勢制御
  {
    // RC入力から目標姿勢を計算
    tar_rpy_.roll =
      dead_zone.inRange(rcin.roll) ? 0 : remap(rcin.roll, -1., 1., -max_attitude_, max_attitude_);
    tar_rpy_.pitch =
      dead_zone.inRange(rcin.pitch) ? 0 : remap(rcin.pitch, -1., 1., -max_attitude_, max_attitude_);

    // 目標水平速度はゼロ
    tar_vel_.x() = 0;
    tar_vel_.y() = 0;
  }

  tar_vel_.z() = remap(rcin.thrust, 0., 1., -max_ver_vel_, max_ver_vel_);
  const auto yawrate =
    dead_zone.inRange(rcin.yaw) ? 0 : remap(rcin.yaw, -1., 1., -max_yawrate_, max_yawrate_);

  // 一度でも上昇コマンドが入力されたら位置制御を行う
  if (is_up_commanded_)
  {
    // 目標速度とヨーレートを積分
    tar_pos_ += tar_vel_ * dt;
    tar_rpy_.yaw += yawrate * dt;
  }
  else
  {
    // 上昇コマンドが入力されるまでは位置とヨーの制御は行わない
    tar_pos_ = odom.frame.p;
    tar_rpy_.yaw = Euler(odom.frame.M).yaw;

    // 上昇コマンドが入力されたかどうかをチェック
    is_up_commanded_ = tar_vel_.z() > 0;
  }

  // コマンドを作成
  const auto cmd = boost::make_shared<tobas_msgs::PoseTwistAccelCommand>();
  cmd->level.data = tobas_msgs::CommandLevel::MANUAL;
  cmd->pos = tar_pos_;
  cmd->vel = tar_vel_;
  cmd->acc.setZero();
  cmd->rpy = tar_rpy_;
  cmd->gyro.setZero();
  cmd->dgyro.setZero();

  // コマンドを発行
  cmd_pub_.publish(cmd);
}

void PoseTwistAccelController::getRosParams(ros::NodeHandle& pnh)
{
  tobas_ros::getParam(
    pnh, "pose_twist_accel/max_horizontal_velocity", max_hor_vel_, kDefaultMaxHorVel,
    tobas_ros::POSITIVE);
  tobas_ros::getParam(
    pnh, "pose_twist_accel/max_vertical_velocity", max_ver_vel_, kDefaultMaxVerVel,
    tobas_ros::POSITIVE);
  tobas_ros::getParam(
    pnh, "pose_twist_accel/max_attitude", max_attitude_, kDefaultMaxAttitude, tobas_ros::POSITIVE);
  tobas_ros::getParam(
    pnh, "pose_twist_accel/max_yawrate", max_yawrate_, kDefaultMaxYawrate, tobas_ros::POSITIVE);
}

void PoseTwistAccelController::registerPublishers(ros::NodeHandle& nh)
{
  cmd_pub_ = nh.advertise<tobas_msgs::PoseTwistAccelCommand>(tobas::kPoseTwistAccelCmdTopic, 1);
}
}  // namespace tobas_rc_teleop

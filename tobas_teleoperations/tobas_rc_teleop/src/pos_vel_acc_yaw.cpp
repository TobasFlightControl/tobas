#include <tobas_std_tools/math.hpp>
#include <tobas_kdl/euler.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/PosVelAccYaw.h>

#include "../include/tobas_rc_teleop/pos_vel_acc_yaw.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;
using namespace KDL;
using namespace tobas_std;

namespace tobas_rc_teleop
{
PosVelAccYawController::PosVelAccYawController(const tobas::Drone& drone) : super(drone)
{
}

void PosVelAccYawController::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  getRosParams(pnh);
  registerPublishers(nh);
}

void PosVelAccYawController::reset(const tobas_msgs::Odometry& odom)
{
  is_up_commanded_ = false;
  t_last_rcin_ = ros::Time::now();
  vel_filter_.initialize(delay_time_const_, Vector::Zero());
  tar_pos_W_ = odom.frame.p;
  tar_vel_F_.setZero();
  tar_yaw_ = Euler(odom.frame.M).yaw;
}

void PosVelAccYawController::update(
  const tobas_msgs::RCInput& rcin,
  const tobas_msgs::Odometry& odom,
  const double&)
{
  // 時刻を更新
  const auto cur_time = ros::Time::now();
  const auto dt = (cur_time - t_last_rcin_).toSec();
  t_last_rcin_ = cur_time;

  // RC入力を速度とヨーレートに変換
  tar_vel_F_.x() =
    dead_zone_.inRange(rcin.pitch) ? 0 : remap(rcin.pitch, -1., 1., -max_hor_vel_, max_hor_vel_);
  tar_vel_F_.y() =
    dead_zone_.inRange(rcin.roll) ? 0 : -remap(rcin.roll, -1., 1., -max_hor_vel_, max_hor_vel_);
  tar_vel_F_.z() = remap(rcin.thrust, 0., 1., -max_ver_vel_, max_ver_vel_);
  const auto yawrate =
    dead_zone_.inRange(rcin.yaw) ? 0 : remap(rcin.yaw, -1., 1., -max_yawrate_, max_yawrate_);

  // 目標速度をフィルタリング
  vel_filter_.update(tar_vel_F_, dt);

  // 目標速度を世界座標系に変換
  // ヨー角の現在値で変換すると直進指令でも進路が曲がってしまうため，指令値で変換する．
  const auto tar_vel_W = Rotation::RotZ(tar_yaw_) * vel_filter_.getState();

  // 目標速度とヨーレートを積分
  tar_pos_W_ += tar_vel_W * dt;
  tar_yaw_ += yawrate * dt;

  // 目標位置とヨー角の偏差を制限
  const auto& cur_pos_W = odom.frame.p;
  const auto cur_yaw = Euler(odom.frame.M).yaw;
  tar_pos_W_ = tar_pos_W_.clamp(cur_pos_W - kMaxPositionError, cur_pos_W + kMaxPositionError);
  tar_yaw_ = clamp(tar_yaw_, cur_yaw - kMaxYawError, cur_yaw + kMaxYawError);

  // 上昇コマンドが入力されるまでは位置とヨーの制御は行わない
  if (!is_up_commanded_)
  {
    tar_pos_W_ = cur_pos_W;
    tar_yaw_ = cur_yaw;
    is_up_commanded_ = tar_vel_F_.z() > 0;
  }

  // コマンドを作成
  const auto cmd = boost::make_shared<tobas_msgs::PosVelAccYaw>();
  cmd->level.data = tobas_msgs::CommandLevel::MANUAL;
  cmd->frame_id.data = tobas_msgs::FrameId::WORLD;
  cmd->pos = tar_pos_W_;
  cmd->vel = tar_vel_W;
  cmd->acc.setZero();
  cmd->yaw = tar_yaw_;

  // コマンドを発行
  cmd_pub_.publish(cmd);
}

void PosVelAccYawController::getRosParams(ros::NodeHandle& pnh)
{
  tobas_ros::getParam(
    pnh, "pos_vel_acc_yaw/max_horizontal_velocity", max_hor_vel_, kDefaultMaxHorVel,
    tobas_ros::POSITIVE);
  tobas_ros::getParam(
    pnh, "pos_vel_acc_yaw/max_vertical_velocity", max_ver_vel_, kDefaultMaxVerVel,
    tobas_ros::POSITIVE);
  tobas_ros::getParam(
    pnh, "pos_vel_acc_yaw/max_yawrate", max_yawrate_, kDefaultMaxYawrate, tobas_ros::POSITIVE);
  tobas_ros::getParam(
    pnh, "pos_vel_acc_yaw/delay_time_const", delay_time_const_, kDefaultDelayTimeConst,
    tobas_ros::NON_NEGATIVE);
}

void PosVelAccYawController::registerPublishers(ros::NodeHandle& nh)
{
  cmd_pub_ = nh.advertise<tobas_msgs::PosVelAccYaw>(tobas::kPosVelAccYawCmdTopic, 1);
}
}  // namespace tobas_rc_teleop

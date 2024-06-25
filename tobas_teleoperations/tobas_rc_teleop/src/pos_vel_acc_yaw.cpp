#include <tobas_kdl/euler.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/PosVelAccYaw.h>

#include "../include/tobas_rc_teleop/pos_vel_acc_yaw.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;
using namespace kdl;

namespace tobas_rc_teleop
{
PosVelAccYawController::PosVelAccYawController(const tobas::Drone& drone) : super(drone)
{
}

void PosVelAccYawController::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  getRosParams(pnh);

  cmd_pub_ = nh.advertise<tobas_msgs::PosVelAccYaw>(tobas::kPosVelAccYawCmdTopic, 1);
}

void PosVelAccYawController::reset(const tobas_msgs::Odometry& odom)
{
  is_up_commanded_ = false;
  t_last_rcin_ = odom.header.stamp;
  tar_pos_W_ = odom.frame.p;
  tar_vel_F_.setZero();
  tar_yaw_ = Euler(odom.frame.M).yaw;
}

void PosVelAccYawController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, const double&)
{
  // 時刻を更新
  const auto dt = (rcin.header.stamp - t_last_rcin_).toSec();
  t_last_rcin_ = rcin.header.stamp;

  // RC入力を速度とヨーレートに変換
  tar_vel_F_.x(remapDead(rcin.pitch, -max_hor_vel_, max_hor_vel_));
  tar_vel_F_.y(-remapDead(rcin.roll, -max_hor_vel_, max_hor_vel_));
  tar_vel_F_.z(remapDead(rcin.throttle, -max_ver_vel_, max_ver_vel_));
  const auto yawrate = remapDead(rcin.yaw, -max_yawrate_, max_yawrate_);

  // 目標速度を世界座標系に変換
  // ヨー角の現在値で変換すると直進指令でも進路が曲がってしまうため，指令値で変換する．
  const auto tar_vel_W = Rotation::RotZ(tar_yaw_) * tar_vel_F_;

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
    pnh, "pos_vel_acc_yaw/max_horizontal_velocity", max_hor_vel_, kDefaultMaxHorVel, tobas_ros::POSITIVE);
  tobas_ros::getParam(
    pnh, "pos_vel_acc_yaw/max_vertical_velocity", max_ver_vel_, kDefaultMaxVerVel, tobas_ros::POSITIVE);
  tobas_ros::getParam(pnh, "pos_vel_acc_yaw/max_yawrate", max_yawrate_, kDefaultMaxYawrate, tobas_ros::POSITIVE);
}
}  // namespace tobas_rc_teleop

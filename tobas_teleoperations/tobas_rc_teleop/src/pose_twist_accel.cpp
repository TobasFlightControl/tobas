#include <tobas_kdl/euler.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/PoseTwistAccelCommand.h>

#include "../include/tobas_rc_teleop/pose_twist_accel.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;

namespace tobas_rc_teleop
{
PoseTwistAccelController::PoseTwistAccelController(const tobas::Drone& drone) : super(drone)
{
}

void PoseTwistAccelController::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  getRosParams(pnh);

  cmd_pub_ = nh.advertise<tobas_msgs::PoseTwistAccelCommand>(tobas::kPoseTwistAccelCmdTopic, 1);
}

void PoseTwistAccelController::reset(const tobas_msgs::Odometry& odom)
{
  is_up_commanded_ = false;
  t_last_rcin_ = odom.header.stamp;
  setToZero(tar_vel_F_);
  tar_pos_W_ = odom.frame.p;
  odom.frame.M.getRPY(tar_rpy_.roll, tar_rpy_.pitch, tar_rpy_.yaw);
}

void PoseTwistAccelController::update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, const double&)
{
  // 時刻を更新
  const auto dt = (rcin.header.stamp - t_last_rcin_).toSec();
  t_last_rcin_ = rcin.header.stamp;

  // GPSwの状態によって水平速度制御モードと姿勢制御モードを切り替える
  if (rcin.gpsw)  // 姿勢固定で位置制御
  {
    // RC入力から目標水平速を計算
    tar_vel_F_.x(remapDead(rcin.pitch, -max_hor_vel_, max_hor_vel_));
    tar_vel_F_.y(-remapDead(rcin.roll, -max_hor_vel_, max_hor_vel_));

    // 目標姿勢角はゼロ
    tar_rpy_.roll = 0;
    tar_rpy_.pitch = 0;
  }
  else  // 位置固定で姿勢制御
  {
    // RC入力から目標姿勢を計算
    tar_rpy_.roll = remapDead(rcin.roll, -max_attitude_, max_attitude_);
    tar_rpy_.pitch = remapDead(rcin.pitch, -max_attitude_, max_attitude_);

    // 目標水平速度はゼロ
    tar_vel_F_.x() = 0;
    tar_vel_F_.y() = 0;
  }

  // RC入力から鉛直速度とヨーレートを計算
  tar_vel_F_.z(remapDead(rcin.throttle, -max_ver_vel_, max_ver_vel_));
  const auto yawrate = remapDead(rcin.yaw, -max_yawrate_, max_yawrate_);

  // 目標速度を世界座標系に変換
  // ヨー角の現在値で変換すると直進指令でも進路が曲がってしまうため，指令値で変換する．
  const auto tar_vel_W = kdl::Rotation::RotZ(tar_rpy_.yaw) * tar_vel_F_;

  // 目標速度とヨーレートを積分
  tar_pos_W_ += tar_vel_W * dt;
  tar_rpy_.yaw += yawrate * dt;

  // 目標位置とヨー角の偏差を制限
  const auto& cur_pos_W = odom.frame.p;
  const auto cur_yaw = kdl::Euler(odom.frame.M).yaw;
  tar_pos_W_ = tar_pos_W_.clamp(cur_pos_W - kMaxPositionError, cur_pos_W + kMaxPositionError);
  tar_rpy_.yaw = clamp(tar_rpy_.yaw, cur_yaw - kMaxYawError, cur_yaw + kMaxYawError);

  // 上昇コマンドが入力されるまでは位置とヨーの制御は行わない
  if (!is_up_commanded_)
  {
    tar_pos_W_ = cur_pos_W;
    tar_rpy_.yaw = cur_yaw;
    is_up_commanded_ = tar_vel_W.z() > 0;
  }

  // コマンドを作成
  const auto cmd = boost::make_shared<tobas_msgs::PoseTwistAccelCommand>();
  cmd->level.data = tobas_msgs::CommandLevel::MANUAL;
  cmd->frame_id.data = tobas_msgs::FrameId::WORLD;
  cmd->pos = tar_pos_W_;
  cmd->vel = tar_vel_W;
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
    pnh, "pose_twist_accel/max_horizontal_velocity", max_hor_vel_, kDefaultMaxHorVel, tobas_ros::POSITIVE);
  tobas_ros::getParam(
    pnh, "pose_twist_accel/max_vertical_velocity", max_ver_vel_, kDefaultMaxVerVel, tobas_ros::POSITIVE);
  tobas_ros::getParam(pnh, "pose_twist_accel/max_attitude", max_attitude_, kDefaultMaxAttitude, tobas_ros::POSITIVE);
  tobas_ros::getParam(pnh, "pose_twist_accel/max_yawrate", max_yawrate_, kDefaultMaxYawrate, tobas_ros::POSITIVE);
}
}  // namespace tobas_rc_teleop

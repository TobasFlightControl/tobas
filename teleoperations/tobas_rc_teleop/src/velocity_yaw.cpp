#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_rc_teleop/velocity_yaw.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;
using namespace Eigen;
using namespace dh_std;

namespace tobas_rc_teleop
{
VelocityYawController::VelocityYawController() : super()
{
}

void VelocityYawController::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  getRosParams(pnh);
  registerPublishers(nh);
}

void VelocityYawController::reset(const tobas_msgs::PoseTwist& pt)
{
  t_last_rcin_ = ros::Time::now();
  vel_filter_.initialize(delay_time_const_, Vector3d::Zero());
  yaw_ = pt.pose.euler.yaw;
}

void VelocityYawController::update(
  const tobas_msgs::RCInput& rcin,
  const dh_std::Range<double>& dead_zone)
{
  // 時刻を更新
  const ros::Time cur_time = ros::Time::now();
  const auto dt = (cur_time - t_last_rcin_).toSec();
  t_last_rcin_ = cur_time;

  // コマンドを作成
  const auto vel_yaw = boost::make_shared<tobas_msgs::VelocityYaw>();
  vel_yaw->level.data = tobas_msgs::CommandLevel::MANUAL;
  vel_yaw->frame_id.data = tobas_msgs::FrameId::GLOBAL;

  // RC入力から速度を計算
  vel_raw_.x() =
    dead_zone.inRange(rcin.pitch) ? 0. : remap(rcin.pitch, -1., 1., -max_hor_vel_, max_hor_vel_);
  vel_raw_.y() =
    dead_zone.inRange(rcin.roll) ? 0. : -remap(rcin.roll, -1., 1., -max_hor_vel_, max_hor_vel_);
  vel_raw_.z() = remap(rcin.thrust, 0., 1., -max_ver_vel_, max_ver_vel_);

  // 速度をフィルタリングしてコマンドに
  vel_filter_.update(vel_raw_, dt);
  vel_yaw->vel.data = vel_filter_.getState();

  // ヨー角を更新
  const auto yawrate =
    dead_zone.inRange(rcin.yaw) ? 0. : remap(rcin.yaw, -1., 1., -max_yawrate_, max_yawrate_);
  yaw_ += yawrate * dt;
  vel_yaw->yaw = yaw_;

  // コマンドを発行
  // 発行後にメッセージが変更されないことを保証するため，コピーへのshared_ptrを作成
  vel_yaw_pub_.publish(vel_yaw);
}

void VelocityYawController::getRosParams(ros::NodeHandle& pnh)
{
  dh_ros::getParam(
    pnh, "velocity_yaw/max_horizontal_velocity", max_hor_vel_, kDefaultMaxHorVel, dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "velocity_yaw/max_vertical_velocity", max_ver_vel_, kDefaultMaxVerVel, dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "velocity_yaw/max_yawrate", max_yawrate_, kDefaultMaxYawrate, dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "velocity_yaw/delay_time_const", delay_time_const_, kDefaultDelayTimeConst,
    dh_ros::NON_NEGATIVE);
}

void VelocityYawController::registerPublishers(ros::NodeHandle& nh)
{
  vel_yaw_pub_ = nh.advertise<tobas_msgs::VelocityYaw>(tobas::kVelocityYawCmdTopic, 1);
}
}  // namespace tobas_rc_teleop

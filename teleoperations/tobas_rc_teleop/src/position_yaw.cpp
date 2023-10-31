#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_rc_teleop/position_yaw.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;
using namespace KDL;
using namespace dh_std;

namespace tobas_rc_teleop
{
PositionYawController::PositionYawController() : super()
{
}

void PositionYawController::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  getRosParams(pnh);

  max_pos_err_.x(max_hor_pos_err_);
  max_pos_err_.y(max_hor_pos_err_);
  max_pos_err_.z(max_ver_pos_err_);

  pos_yaw_.level.data = tobas_msgs::CommandLevel::MANUAL;

  registerPublishers(nh);
}

void PositionYawController::reset(const tobas_msgs::PoseTwist& pt)
{
  pos_yaw_.pos = pt.pose.pos;
  pos_yaw_.yaw = pt.pose.euler.yaw;
  t_last_rcin_ = ros::Time::now();
}

void PositionYawController::update(
  const tobas_msgs::RCInput& rcin,
  const dh_std::Range<double>& dead_zone,
  const Vector& cur_pos,
  const double& cur_yaw)
{
  const ros::Time cur_time = ros::Time::now();
  const auto dt = (cur_time - t_last_rcin_).toSec();
  t_last_rcin_ = cur_time;

  // 位置とヨー角の変化率を計算
  vel_.x(
    dead_zone.inRange(rcin.pitch) ? 0. : remap(rcin.pitch, -1., 1., -max_hor_vel_, max_hor_vel_));
  vel_.y(
    dead_zone.inRange(rcin.roll) ? 0. : -remap(rcin.roll, -1., 1., -max_hor_vel_, max_hor_vel_));
  vel_.z(remap(rcin.thrust, 0., 1., -max_ver_vel_, max_ver_vel_));  // スラストレバーに遊びはなし
  const auto yawrate =
    dead_zone.inRange(rcin.yaw) ? 0. : remap(rcin.yaw, -1., 1., -max_yawrate_, max_yawrate_);

  // コマンドを更新
  pos_yaw_.pos += vel_ * dt;
  pos_yaw_.yaw += yawrate * dt;

  // 誤差を制限
  pos_yaw_.pos = pos_yaw_.pos.clamp(cur_pos - max_pos_err_, cur_pos + max_pos_err_);
  pos_yaw_.yaw = clamp(pos_yaw_.yaw, cur_yaw - max_yaw_err_, cur_yaw + max_yaw_err_);

  // コマンドを発行
  // 発行後にメッセージが変更されないことを保証するため，コピーへのshared_ptrを作成
  const auto pos_yaw_ptr = boost::make_shared<tobas_msgs::PositionYaw>(pos_yaw_);
  pos_yaw_pub_.publish(pos_yaw_ptr);
}

void PositionYawController::getRosParams(ros::NodeHandle& pnh)
{
  dh_ros::getParam(
    pnh, "position_yaw/max_horizontal_velocity", max_hor_vel_, kDefaultMaxHorVel, dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "position_yaw/max_vertical_velocity", max_ver_vel_, kDefaultMaxVerVel, dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "position_yaw/max_yawrate", max_yawrate_, kDefaultMaxYawrate, dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "pos_vel_acc_yaw/max_horizontal_position_error", max_hor_pos_err_, kDefaultMaxHorPosErr,
    dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "pos_vel_acc_yaw/max_vertical_position_error", max_ver_pos_err_, kDefaultMaxVerPosErr,
    dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "pos_vel_acc_yaw/max_yaw_error", max_yaw_err_, kDefaultMaxYawErr, dh_ros::POSITIVE);
}

void PositionYawController::registerPublishers(ros::NodeHandle& nh)
{
  pos_yaw_pub_ = nh.advertise<tobas_msgs::PositionYaw>(tobas::kPositionYawCmdTopic, 1);
}
}  // namespace tobas_rc_teleop

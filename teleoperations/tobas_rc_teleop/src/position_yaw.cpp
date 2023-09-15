#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include "../include/tobas_rc_teleop/position_yaw.hpp"

using namespace std;
using namespace dh_std;

namespace tobas_rc_teleop
{
PositionYawController::PositionYawController() : super()
{
}

void PositionYawController::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  pos_yaw_.level.data = tobas_msgs::CommandLevel::MANUAL;

  getRosParams(pnh);
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
  const dh_std::Range<double>& dead_zone)
{
  const ros::Time cur_time = ros::Time::now();
  const auto dt = (cur_time - t_last_rcin_).toSec();
  t_last_rcin_ = cur_time;

  // 位置とヨー角の変化率を計算
  vel_.x(
    dead_zone.inRange(rcin.pitch) ? 0. : remap(rcin.pitch, -1., 1., -max_hor_vel_, max_hor_vel_));
  vel_.y(
    dead_zone.inRange(rcin.roll) ? 0. : -remap(rcin.roll, -1., 1., -max_hor_vel_, max_hor_vel_));
  vel_.z(
    dead_zone.inRange(rcin.thrust) ? 0. : remap(rcin.thrust, -1., 1., -max_ver_vel_, max_ver_vel_));
  const auto yawrate =
    dead_zone.inRange(rcin.yaw) ? 0. : remap(rcin.yaw, -1., 1., -max_yawrate_, max_yawrate_);

  // コマンドを更新
  pos_yaw_.pos += vel_ * dt;
  pos_yaw_.yaw += yawrate * dt;

  // コマンドを発行
  // 発行後にメッセージが変更されないことを保証するため，コピーへのshared_ptrを作成
  const auto pos_yaw_ptr = boost::make_shared<tobas_msgs::PositionYaw>(pos_yaw_);
  pos_yaw_pub_.publish(pos_yaw_ptr);
}

void PositionYawController::getRosParams(ros::NodeHandle& pnh)
{
  dh_ros::getParam(
    pnh, "position_yaw/max_horizontal_velocity", max_hor_vel_, kDefaultMaxHorizontalVelocity,
    dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "position_yaw/max_vertical_velocity", max_ver_vel_, kDefaultMaxVerticalVelocity,
    dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "position_yaw/max_yawrate", max_yawrate_, kDefaultMaxYawrate, dh_ros::POSITIVE);
}

void PositionYawController::registerPublishers(ros::NodeHandle& nh)
{
  pos_yaw_pub_ = nh.advertise<tobas_msgs::PositionYaw>("command/position_yaw", 1);
}
}  // namespace tobas_rc_teleop

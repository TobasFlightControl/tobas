#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include "../include/tobas_rc_teleop/velocity_yaw.hpp"

using namespace std;
using namespace dh_std;

namespace tobas_rc_teleop
{
VelocityYawController::VelocityYawController() : super()
{
}

void VelocityYawController::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  vel_yaw_.level.data = tobas_msgs::CommandLevel::MANUAL;
  vel_yaw_.frame_id.data = tobas_msgs::FrameId::GLOBAL;

  getRosParams(pnh);
  registerPublishers(nh);
}

void VelocityYawController::reset(const tobas_msgs::PoseTwist& pt)
{
  vel_yaw_.yaw = pt.pose.euler.yaw;  // 最初は現在のヨー角を指令
  t_last_rcin_ = ros::Time::now();
}

void VelocityYawController::update(
  const tobas_msgs::RCInput& rcin,
  const dh_std::Range<double>& dead_zone)
{
  // 並進速度を更新
  vel_yaw_.vel.x(
    dead_zone.inRange(rcin.pitch) ? 0. : remap(rcin.pitch, -1., 1., -max_hor_vel_, max_hor_vel_));
  vel_yaw_.vel.y(
    dead_zone.inRange(rcin.roll) ? 0. : -remap(rcin.roll, -1., 1., -max_hor_vel_, max_hor_vel_));
  vel_yaw_.vel.z(remap(rcin.thrust, 0., 1., -max_ver_vel_, max_ver_vel_));

  // Yawの目標値を更新
  const ros::Time cur_time = ros::Time::now();
  const auto dt = (cur_time - t_last_rcin_).toSec();
  t_last_rcin_ = cur_time;
  const auto yawrate =
    dead_zone.inRange(rcin.yaw) ? 0. : remap(rcin.yaw, -1., 1., -max_yawrate_, max_yawrate_);
  vel_yaw_.yaw += yawrate * dt;

  // コマンドを発行
  // 発行後にメッセージが変更されないことを保証するため，コピーへのshared_ptrを作成
  const auto vel_yaw_ptr = boost::make_shared<tobas_msgs::VelocityYaw>(vel_yaw_);
  vel_yaw_pub_.publish(vel_yaw_ptr);
}

void VelocityYawController::getRosParams(ros::NodeHandle& pnh)
{
  dh_ros::getParam(
    pnh, "max_horizontal_velocity", max_hor_vel_, kDefaultMaxHorizontalVelocity, dh_ros::POSITIVE);
  dh_ros::getParam(
    pnh, "max_vertical_velocity", max_ver_vel_, kDefaultMaxVerticalVelocity, dh_ros::POSITIVE);
  dh_ros::getParam(pnh, "max_yawrate", max_yawrate_, kDefaultMaxYawrate, dh_ros::POSITIVE);
}

void VelocityYawController::registerPublishers(ros::NodeHandle& nh)
{
  vel_yaw_pub_ = nh.advertise<tobas_msgs::VelocityYaw>("command/velocity_yaw", 1);
}
}  // namespace tobas_rc_teleop

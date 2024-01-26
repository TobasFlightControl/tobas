#include <tobas_std_tools/math.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_rc_teleop/position_yaw.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;
using namespace KDL;
using namespace tobas_std;

namespace tobas_rc_teleop
{
PositionYawController::PositionYawController(const tobas::Drone& drone) : super(drone)
{
}

void PositionYawController::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  getRosParams(pnh);
  pos_yaw_.level.data = tobas_msgs::CommandLevel::MANUAL;
  registerPublishers(nh);
}

void PositionYawController::reset(const tobas_msgs::Odometry& odom)
{
  pos_yaw_.pos = odom.pose.pos;
  pos_yaw_.yaw = odom.pose.euler.yaw;
  t_last_rcin_ = ros::Time::now();
}

void PositionYawController::update(
  const tobas_msgs::RCInput& rcin,
  const tobas_msgs::Odometry&,
  const double&,
  const Range<double>& dead_zone)
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
    dead_zone.inRange(rcin.yaw) ? 0 : remap(rcin.yaw, -1., 1., -max_yawrate_, max_yawrate_);

  // コマンドを更新
  pos_yaw_.pos += vel_ * dt;
  pos_yaw_.yaw += yawrate * dt;

  // コマンドを制限
  pos_yaw_.pos.z() = clamp(pos_yaw_.pos.z(), min_alt_, max_alt_);  // 高度制限

  // コマンドを発行
  // 発行後にメッセージが変更されないことを保証するため，コピーへのshared_ptrを作成
  const auto pos_yaw_ptr = boost::make_shared<tobas_msgs::PositionYaw>(pos_yaw_);
  pos_yaw_pub_.publish(pos_yaw_ptr);
}

void PositionYawController::getRosParams(ros::NodeHandle& pnh)
{
  tobas_ros::getParam(
    pnh, "pose_twist_accel/min_altitude", min_alt_, kDefaultMinAltitude, tobas_ros::NON_POSITIVE);
  tobas_ros::getParam(
    pnh, "pose_twist_accel/max_altitude", max_alt_, kDefaultMaxAltitude, tobas_ros::POSITIVE);
  if (min_alt_ >= max_alt_)
    ROS_THROW("The maximum target altitude must be greater than minimum target altitude.");

  tobas_ros::getParam(
    pnh, "position_yaw/max_horizontal_velocity", max_hor_vel_, kDefaultMaxHorVel,
    tobas_ros::POSITIVE);
  tobas_ros::getParam(
    pnh, "position_yaw/max_vertical_velocity", max_ver_vel_, kDefaultMaxVerVel,
    tobas_ros::POSITIVE);
  tobas_ros::getParam(
    pnh, "position_yaw/max_yawrate", max_yawrate_, kDefaultMaxYawrate, tobas_ros::POSITIVE);
}

void PositionYawController::registerPublishers(ros::NodeHandle& nh)
{
  pos_yaw_pub_ = nh.advertise<tobas_msgs::PositionYaw>(tobas::kPositionYawCmdTopic, 1);
}
}  // namespace tobas_rc_teleop

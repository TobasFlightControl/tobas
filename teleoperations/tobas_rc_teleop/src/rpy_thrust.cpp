#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_tools/utils.hpp>
#include <tobas_msgs/RollPitchYawThrust.h>

#include "../include/tobas_rc_teleop/rpy_thrust.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;
using namespace dh_std;

namespace tobas_rc_teleop
{
RollPitchYawThrustController::RollPitchYawThrustController()
  : super(), z_rotors_(drone_, tobas::Axis::Z_POSITIVE)
{
}

void RollPitchYawThrustController::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  getRosParams(pnh);

  drone_.loadFromParam(nh);
  z_rotors_.updateInternalDataStructures();

  const auto mass = tobas::getMass();
  max_thrust_ = mass * (tobas::kGravity + max_ver_acc_);

  registerPublishers(nh);
}

void RollPitchYawThrustController::reset(const tobas_msgs::PoseTwist& pt)
{
  yaw_ = pt.pose.euler.yaw;
  t_last_rcin_ = ros::Time::now();
}

void RollPitchYawThrustController::getRosParams(ros::NodeHandle& pnh)
{
  dh_ros::getParam(pnh, "max_attitude", max_attitude_, kDefaultMaxAttitude, dh_ros::POSITIVE);
  dh_ros::getParam(pnh, "max_yawrate", max_yawrate_, kDefaultMaxYawrate, dh_ros::POSITIVE);

  dh_ros::getParam(
    pnh, "max_vertical_accel", max_ver_acc_, kDefaultMaxVerticalAccel, dh_ros::POSITIVE);
  if (max_ver_acc_ >= tobas::kGravity)
  {
    rosthrow(kControllerName, "Maximum vertical acceleration must be lower than gravity.");
  }
}

void RollPitchYawThrustController::registerPublishers(ros::NodeHandle& nh)
{
  rpy_thrust_pub_ = nh.advertise<tobas_msgs::RollPitchYawThrust>("command/rpy_thrust", 1);
}

void RollPitchYawThrustController::update(
  const tobas_msgs::RCInput& rcin,
  const double& battery_voltage,
  const dh_std::Range<double>& dead_zone)
{
  assert(battery_voltage > 0.);

  // Yawの目標値を更新
  const ros::Time cur_time = ros::Time::now();
  const auto dt = (cur_time - t_last_rcin_).toSec();
  t_last_rcin_ = cur_time;
  const auto yawrate =
    dead_zone.inRange(rcin.yaw) ? 0. : remap(rcin.yaw, -1., 1., -max_yawrate_, max_yawrate_);
  yaw_ += yawrate * dt;

  // コマンドを作成
  const auto rpyt = boost::make_shared<tobas_msgs::RollPitchYawThrust>();
  rpyt->level.data = tobas_msgs::CommandLevel::MANUAL;

  // 姿勢と推力を埋める
  rpyt->rpy.roll =
    dead_zone.inRange(rcin.roll) ? 0. : remap(rcin.roll, -1., 1., -max_attitude_, max_attitude_);
  rpyt->rpy.pitch =
    dead_zone.inRange(rcin.pitch) ? 0. : remap(rcin.pitch, -1., 1., -max_attitude_, max_attitude_);
  rpyt->rpy.yaw = yaw_;

  const auto min_thrust = z_rotors_.minThrustSum(battery_voltage);
  const auto max_thrust = min(max_thrust_, z_rotors_.maxThrustSum(battery_voltage));
  rpyt->thrust = remap(rcin.thrust, 0., 1., min_thrust, max_thrust);

  // コマンドを発行
  rpy_thrust_pub_.publish(rpyt);
}
}  // namespace tobas_rc_teleop

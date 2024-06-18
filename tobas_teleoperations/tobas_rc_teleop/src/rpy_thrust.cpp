#include <tobas_kdl/euler.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_tools/utils.hpp>
#include <tobas_msgs/RollPitchYawThrust.h>

#include "../include/tobas_rc_teleop/rpy_thrust.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;

namespace tobas_rc_teleop
{
RollPitchYawThrustController::RollPitchYawThrustController(const tobas::Drone& drone)
  : super(drone), z_rotors_(drone_, tobas::Axis::Z_POSITIVE)
{
}

void RollPitchYawThrustController::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  getRosParams(pnh);

  z_rotors_.updateInternalDataStructures();

  const auto mass = tobas::getMass();
  max_thrust_ = mass * (tobas::kGravity + max_ver_acc_);

  rpy_thrust_pub_ = nh.advertise<tobas_msgs::RollPitchYawThrust>(tobas::kRpyThrustCmdTopic, 1);
}

void RollPitchYawThrustController::reset(const tobas_msgs::Odometry& odom)
{
  yaw_ = tobas_kdl::Euler(odom.frame.M).yaw;
  t_last_rcin_ = odom.header.stamp;
}

void RollPitchYawThrustController::update(
  const tobas_msgs::RCInput& rcin,
  const tobas_msgs::Odometry&,
  const double& battery_voltage)
{
  assert(battery_voltage > 0);

  // 時刻を更新
  const auto dt = (rcin.header.stamp - t_last_rcin_).toSec();
  t_last_rcin_ = rcin.header.stamp;

  // Yawの目標値を更新
  const auto yawrate = remapDead(rcin.yaw, -max_yawrate_, max_yawrate_);
  yaw_ += yawrate * dt;

  // コマンドを作成
  const auto rpyt = boost::make_shared<tobas_msgs::RollPitchYawThrust>();
  rpyt->level.data = tobas_msgs::CommandLevel::MANUAL;

  // 姿勢と推力を埋める
  rpyt->rpy.roll = remapDead(rcin.roll, -max_attitude_, max_attitude_);
  rpyt->rpy.pitch = remapDead(rcin.pitch, -max_attitude_, max_attitude_);
  rpyt->rpy.yaw = yaw_;

  const auto min_thrust = z_rotors_.minThrustSum(battery_voltage);
  const auto max_thrust = min(max_thrust_, z_rotors_.maxThrustSum(battery_voltage));
  rpyt->thrust = remap(rcin.throttle, min_thrust, max_thrust);

  // コマンドを発行
  rpy_thrust_pub_.publish(rpyt);
}

void RollPitchYawThrustController::getRosParams(ros::NodeHandle& pnh)
{
  tobas_ros::getParam(pnh, "rpy_thrust/max_attitude", max_attitude_, kDefaultMaxAttitude, tobas_ros::POSITIVE);
  tobas_ros::getParam(pnh, "rpy_thrust/max_yawrate", max_yawrate_, kDefaultMaxYawrate, tobas_ros::POSITIVE);
  tobas_ros::getParam(pnh, "rpy_thrust/max_vertical_accel", max_ver_acc_, kDefaultMaxVerAcc, tobas_ros::POSITIVE);
}
}  // namespace tobas_rc_teleop

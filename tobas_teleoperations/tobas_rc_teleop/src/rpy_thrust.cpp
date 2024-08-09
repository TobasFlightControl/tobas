#include <tobas_kdl/euler.hpp>

#include <tobas_constants/constants.hpp>

#include <tobas_msgs/msg/roll_pitch_yaw_thrust.hpp>

#include "../include/tobas_rc_teleop/rpy_thrust.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;

namespace tobas_rc_teleop
{
RollPitchYawThrustController::RollPitchYawThrustController(const tobas::Drone& drone)
  : super(drone), z_rotors_(drone_, tobas::Z_POSITIVE)
{
}

void RollPitchYawThrustController::initialize()
{
  getRosParams(pnh);

  z_rotors_.updateInternalDataStructures();

  const auto mass = tobas::getMass();
  max_thrust_ = mass * (tobas_std::kGravity + max_ver_acc_);

  rpy_thrust_pub_ = node.advertise<tobas_msgs::RollPitchYawThrust>(tobas::kRpyThrustCmdTopic);
}

void RollPitchYawThrustController::reset(const tobas_msgs::Odometry& odom)
{
  yaw_ = kdl::Euler(odom.frame.M).yaw;
  t_last_rcin_ = odom.header.stamp;
}

void RollPitchYawThrustController::update(
  const tobas_msgs::RCInput& rcin,
  const tobas_msgs::Odometry&,
  const double& battery_voltage)
{
  assert(battery_voltage > 0);

  // 時刻を更新
  const auto dt = (rcin.header.stamp - t_last_rcin_).seconds();
  t_last_rcin_ = rcin.header.stamp;

  // Yawの目標値を更新
  const auto yawrate = remapDead(rcin.yaw, -max_yawrate_, max_yawrate_);
  yaw_ += yawrate * dt;

  // コマンドを作成
  const auto rpyt =std::make_unique<tobas_msgs::RollPitchYawThrust>();
  rpyt->level.data = tobas_msgs::CommandLevel::MANUAL;

  // 姿勢と推力を埋める
  rpyt->rpy.roll = remapDead(rcin.roll, -max_attitude_, max_attitude_);
  rpyt->rpy.pitch = remapDead(rcin.pitch, -max_attitude_, max_attitude_);
  rpyt->rpy.yaw = yaw_;

  const auto min_thrust = z_rotors_.minThrustSum(battery_voltage);
  const auto max_thrust = min(max_thrust_, z_rotors_.maxThrustSum(battery_voltage));
  rpyt->thrust = remap(rcin.throttle, min_thrust, max_thrust);

  // コマンドを発行
  rpy_thrust_pub_->publish(rpyt);
}

void RollPitchYawThrustController::getRosParams(rclcpp::Node::SharedPtr pnh)
{
  ros2::getParam(pnh, "rpy_thrust/max_attitude", max_attitude_, kDefaultMaxAttitude, ros2::POSITIVE);
  ros2::getParam(pnh, "rpy_thrust/max_yawrate", max_yawrate_, kDefaultMaxYawrate, ros2::POSITIVE);
  ros2::getParam(pnh, "rpy_thrust/max_vertical_accel", max_ver_acc_, kDefaultMaxVerAcc, ros2::POSITIVE);
}
}  // namespace tobas_rc_teleop

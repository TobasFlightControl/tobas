#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_tools/utils.hpp>
#include <tobas_msgs/RollPitchYawrateThrust.h>

#include "../include/tobas_rc_teleop/roll_pitch_yawrate_thrust.hpp"

using namespace std;
using namespace dh_std;

namespace tobas_rc_teleop
{
RollPitchYawrateThrustController::RollPitchYawrateThrustController()
  : super(), z_rotors_(drone_, tobas::Axis::Z_POSITIVE)
{
}

void RollPitchYawrateThrustController::initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  getRosParams(pnh);

  drone_.loadFromParam(nh);
  z_rotors_.updateInternalDataStructures();

  const auto mass = tobas::getMass();
  max_thrust_ = mass * (tobas::kGravity + max_acc_);
  min_thrust_ = mass * (tobas::kGravity + min_acc_);

  registerPublishers(nh);
}

void RollPitchYawrateThrustController::reset()
{
}

void RollPitchYawrateThrustController::getRosParams(ros::NodeHandle& pnh)
{
  dh_ros::getParam(pnh, "max_attitude", max_attitude_, kDefaultMaxAttitude, dh_ros::POSITIVE);
  dh_ros::getParam(pnh, "max_yawrate", max_yawrate_, kDefaultMaxYawrate, dh_ros::POSITIVE);

  dh_ros::getParam(pnh, "max_acceleration", max_acc_, kDefaultMaxAcceleration, dh_ros::POSITIVE);
  dh_ros::getParam(pnh, "min_acceleration", min_acc_, kDefaultMinAcceleration, dh_ros::NEGATIVE);
  if (min_acc_ < -tobas::kGravity)
  {
    rosthrow(kControllerName, "'min_acceleration' must be greater than -Gravity.");
  }
}

void RollPitchYawrateThrustController::registerPublishers(ros::NodeHandle& nh)
{
  rpydt_pub_ =
    nh.advertise<tobas_msgs::RollPitchYawrateThrust>("command/roll_pitch_yawrate_thrust", 1);
}

void RollPitchYawrateThrustController::update(
  const tobas_msgs::RCInput& rcin,
  const double& battery_voltage,
  const dh_std::Range<double>& dead_zone)
{
  assert(battery_voltage > 0.);

  // コマンドを作成
  const auto rpydt = boost::make_shared<tobas_msgs::RollPitchYawrateThrust>();
  rpydt->level.data = tobas_msgs::CommandLevel::MANUAL;  // 最大優先順位
  rpydt->roll =
    dead_zone.inRange(rcin.roll) ? 0. : remap(rcin.roll, -1., 1., -max_attitude_, max_attitude_);
  rpydt->pitch =
    dead_zone.inRange(rcin.pitch) ? 0. : remap(rcin.pitch, -1., 1., -max_attitude_, max_attitude_);
  rpydt->yawrate =
    dead_zone.inRange(rcin.yaw) ? 0. : remap(rcin.yaw, -1., 1., -max_yawrate_, max_yawrate_);

  const auto min_thrust = max(min_thrust_, z_rotors_.minThrustSum(battery_voltage));
  const auto max_thrust = min(max_thrust_, z_rotors_.maxThrustSum(battery_voltage));
  rpydt->thrust = remap(rcin.thrust, 0., 1., min_thrust, max_thrust);

  // コマンドを発行
  rpydt_pub_.publish(rpydt);
}
}  // namespace tobas_rc_teleop

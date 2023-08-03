#include <dh_ros_tools/rosparam.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_tools/utils.hpp>

#include "../../include/tobas_rc_teleop/roll_pitch_yawrate_thrust.hpp"
#include "../../include/tobas_rc_teleop/common.hpp"

using namespace std;
using namespace dh_std;

namespace tobas_rc_teleop
{
RcinToRollPitchYawrateThrust::RcinToRollPitchYawrateThrust()
  : super(),
    z_rotors_(drone_, tobas::Axis::Z_POSITIVE),
    battery_received_(false),
    rcin_received_(false),
    last_toggle_(false)
{
  getRosParams();

  drone_.loadFromParam(ns_);
  z_rotors_.updateInternalDataStructures();

  const auto mass = tobas::getMass();
  max_thrust_ = mass * (tobas::kGravity + max_acc_);
  min_thrust_ = mass * (tobas::kGravity + min_acc_);

  dead_zone_.lower = -dead_zone_rate_ / 2;
  dead_zone_.upper = dead_zone_rate_ / 2;

  // プロポによる制御を最大の優先順位に設定
  rpydt_.level.data = tobas_msgs::CommandLevel::MANUAL;

  registerPublishers();
  registerSubscribers();
}

void RcinToRollPitchYawrateThrust::getRosParams()
{
  dh_ros::getParam("~max_attitude", max_attitude_, kDefaultMaxAttitude, dh_ros::POSITIVE);
  dh_ros::getParam("~max_yawrate", max_yawrate_, kDefaultMaxYawrate, dh_ros::POSITIVE);

  dh_ros::getParam("~max_acceleration", max_acc_, kDefaultMaxAcceleration, dh_ros::POSITIVE);
  dh_ros::getParam("~min_acceleration", min_acc_, kDefaultMinAcceleration, dh_ros::NEGATIVE);
  if (min_acc_ < -tobas::kGravity)
  {
    rosthrow("'min_acceleration' must be greater than -Gravity.");
  }

  dh_ros::getParam("~dead_zone_rate", dead_zone_rate_, kDefaultDeadZoneRate, dh_ros::NON_NEGATIVE);
  if (dead_zone_rate_ >= 1.)
  {
    rosthrow("'dead_zone_rate' must be lower than 1.");
  }
}

void RcinToRollPitchYawrateThrust::registerPublishers()
{
  rpydt_pub_ =
    nh_.advertise<tobas_msgs::RollPitchYawrateThrust>("command/roll_pitch_yawrate_thrust", 1);
  event_pub_ = nh_.advertise<tobas_msgs::Event>("event", 1);
}

void RcinToRollPitchYawrateThrust::registerSubscribers()
{
  battery_sub_ = nh_.subscribe("battery", 1, &RcinToRollPitchYawrateThrust::batteryCb, this);
  rcin_sub_ = nh_.subscribe("rc_input", 1, &RcinToRollPitchYawrateThrust::rcInputCb, this);
}

void RcinToRollPitchYawrateThrust::requestShutdown()
{
  event_.data = tobas_msgs::Event::SHUTDOWN;
  event_pub_.publish(event_);
}

void RcinToRollPitchYawrateThrust::eventCb(const tobas_msgs::Event& event)
{
  switch (event.data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      ros::shutdown();
      break;
    default:
      break;
  }
}

void RcinToRollPitchYawrateThrust::batteryCb(const tobas_msgs::Battery& battery)
{
  battery_ = battery;

  if (!battery_received_)
  {
    battery_received_ = true;
  }
}

void RcinToRollPitchYawrateThrust::rcInputCb(const tobas_msgs::RCInput& rcin)
{
  // バッテリーの状態が取得できていなければ動かさない
  if (!battery_received_)
  {
    return;
  }

  // トグルをオフにした状態で起動することを要求
  if (!rcin_received_)
  {
    if (rcin.toggle)
    {
      rosErrorThrottle(
        kErrorPeriod, "Please start with the transmitter's toggle in the OFF position.");
      return;
    }
    rcin_received_ = true;
  }

  // トグル変化時の処理
  // ON -> OFF: 強制シャットダウン (最終手段)
  if (last_toggle_ && !rcin.toggle)
  {
    rosInfo("The toggle has changed from ON to OFF. Shutting down the system.");
    requestShutdown();
    return;
  }

  // コマンドを更新
  rpydt_.roll =
    dead_zone_.inRange(rcin.roll) ? 0. : remap(rcin.roll, -1., 1., -max_attitude_, max_attitude_);
  rpydt_.pitch =
    dead_zone_.inRange(rcin.pitch) ? 0. : remap(rcin.pitch, -1., 1., -max_attitude_, max_attitude_);
  rpydt_.yawrate =
    dead_zone_.inRange(rcin.yaw) ? 0. : remap(rcin.yaw, -1., 1., -max_yawrate_, max_yawrate_);

  const auto min_thrust = max(min_thrust_, z_rotors_.minThrustSum(battery_.voltage));
  const auto max_thrust = min(max_thrust_, z_rotors_.maxThrustSum(battery_.voltage));
  rpydt_.thrust = remap(rcin.thrust, 0., 1., min_thrust, max_thrust);

  // コマンドを発行
  rpydt_pub_.publish(rpydt_);
}
}  // namespace tobas_rc_teleop

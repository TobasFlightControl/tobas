#include <dh_ros_tools/rosparam.hpp>

#include "../../include/tobas_rc_teleop/rcin2rpydt.hpp"

namespace tobas_rc_teleop
{
RcinToRollPitchYawrateThrust::RcinToRollPitchYawrateThrust()
  : super(), z_rotors_(drone_, tobas::Axis::Z_POSITIVE), battery_received_(false)
{
  getRosParams();

  drone_.loadFromParam(ns_);
  z_rotors_.updateInternalDataStructures();

  // プロポによる制御を最大の優先順位に設定
  rpydt_.level.data = tobas_msgs::CommandLevel::MANUAL;

  registerPublishers();
  registerSubscribers();
}

void RcinToRollPitchYawrateThrust::getRosParams()
{
  dh_ros::getParam("~max_attitude", max_attitude_, kDefaultMaxAttitude, dh_ros::POSITIVE);
  dh_ros::getParam("~max_yawrate", max_yawrate_, kDefaultMaxYawrate, dh_ros::POSITIVE);
}

void RcinToRollPitchYawrateThrust::registerPublishers()
{
  rpydt_pub_ =
    nh_.advertise<tobas_msgs::RollPitchYawrateThrust>("command/roll_pitch_yawrate_thrust", 1);
}

void RcinToRollPitchYawrateThrust::registerSubscribers()
{
  battery_sub_ = nh_.subscribe("battery", 1, &RcinToRollPitchYawrateThrust::batteryCb, this);
  rcin_sub_ = nh_.subscribe("rc_input", 1, &RcinToRollPitchYawrateThrust::rcInputCb, this);
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
  if (!battery_received_)
  {
    return;
  }

  if (!rcin.toggle)
  {
    return;
  }

  // コマンドを更新
  rpydt_.roll = dh_std::remap(rcin.roll, -1., 1., -max_attitude_, max_attitude_);
  rpydt_.pitch = dh_std::remap(rcin.pitch, -1., 1., -max_attitude_, max_attitude_);
  rpydt_.yawrate = dh_std::remap(rcin.yaw, -1., 1., -max_yawrate_, max_yawrate_);
  rpydt_.thrust = z_rotors_.maxThrustSum(battery_.voltage) * rcin.throttle;

  // コマンドを発行
  rpydt_pub_.publish(rpydt_);
}
}  // namespace tobas_rc_teleop

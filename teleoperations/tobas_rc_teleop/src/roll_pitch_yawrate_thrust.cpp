#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_tools/utils.hpp>

#include "../include/tobas_rc_teleop/roll_pitch_yawrate_thrust.hpp"
#include "../include/tobas_rc_teleop/common.hpp"

using namespace std;
using namespace dh_std;

namespace tobas_rc_teleop
{
RcinToRollPitchYawrateThrust::RcinToRollPitchYawrateThrust(
  ros::NodeHandle nh,
  ros::NodeHandle pnh,
  string name)
  : super(nh, pnh, name), z_rotors_(drone_, tobas::Axis::Z_POSITIVE)
{
  getRosParams();

  drone_.loadFromParam(nh_);
  z_rotors_.updateInternalDataStructures();

  const auto mass = tobas::getMass();
  max_thrust_ = mass * (tobas::kGravity + max_acc_);
  min_thrust_ = mass * (tobas::kGravity + min_acc_);

  dead_zone_.lower = -dead_zone_rate_ / 2;
  dead_zone_.upper = dead_zone_rate_ / 2;

  registerPublishers();
  registerSubscribers();
}

void RcinToRollPitchYawrateThrust::getRosParams()
{
  dh_ros::getParam(pnh_, "max_attitude", max_attitude_, kDefaultMaxAttitude, dh_ros::POSITIVE);
  dh_ros::getParam(pnh_, "max_yawrate", max_yawrate_, kDefaultMaxYawrate, dh_ros::POSITIVE);

  dh_ros::getParam(pnh_, "max_acceleration", max_acc_, kDefaultMaxAcceleration, dh_ros::POSITIVE);
  dh_ros::getParam(pnh_, "min_acceleration", min_acc_, kDefaultMinAcceleration, dh_ros::NEGATIVE);
  if (min_acc_ < -tobas::kGravity)
  {
    rosthrow(name_, "'min_acceleration' must be greater than -Gravity.");
  }

  dh_ros::getParam(
    pnh_, "dead_zone_rate", dead_zone_rate_, kDefaultDeadZoneRate, dh_ros::NON_NEGATIVE);
  if (dead_zone_rate_ >= 1.)
  {
    rosthrow(name_, "'dead_zone_rate' must be lower than 1.");
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
  battery_sub_ =
    nh_.subscribe("battery", 1, &RcinToRollPitchYawrateThrust::batteryCb, this, tcpNoDelay());
  rcin_sub_ =
    nh_.subscribe("rc_input", 1, &RcinToRollPitchYawrateThrust::rcInputCb, this, tcpNoDelay());
}

void RcinToRollPitchYawrateThrust::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      nh_.shutdown();
      break;
    default:
      break;
  }
}

void RcinToRollPitchYawrateThrust::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  battery_ = battery;
}

void RcinToRollPitchYawrateThrust::rcInputCb(const tobas_msgs::RCInputConstPtr& rcin)
{
  switch (stage_)
  {
    case CHECK_PREREQUISITES:
    {
      if (battery_ != nullptr)
      {
        stage_ = FIRST_RCIN;
      }
      break;
    }

    case FIRST_RCIN:
    {
      if (rcin->toggle)
      {
        rosErrorThrottle(
          kErrorPeriod, name_, "Please start with the transmitter's toggle in the OFF position.");
      }
      else
      {
        rosInfo(name_, "RC transmitter is ready. Toggle on to start control.");
        stage_ = TOGGLE_OFF;
      }
      break;
    }

    case TOGGLE_OFF:
    {
      if (rcin->toggle)
      {
        stage_ = TOGGLE_ON;
      }
      break;
    }

    case TOGGLE_ON:
    {
      if (!rcin->toggle)
      {
        rosInfo(name_, "The toggle has changed from ON to OFF. Shutting down the system.");
        requestShutdown();
        nh_.shutdown();
      }

      // コマンドを作成
      const auto rpydt = boost::make_shared<tobas_msgs::RollPitchYawrateThrust>();
      rpydt->level.data = tobas_msgs::CommandLevel::MANUAL;  // 最大優先順位
      rpydt->roll = dead_zone_.inRange(rcin->roll) ?
                      0. :
                      remap(rcin->roll, -1., 1., -max_attitude_, max_attitude_);
      rpydt->pitch = dead_zone_.inRange(rcin->pitch) ?
                       0. :
                       remap(rcin->pitch, -1., 1., -max_attitude_, max_attitude_);
      rpydt->yawrate =
        dead_zone_.inRange(rcin->yaw) ? 0. : remap(rcin->yaw, -1., 1., -max_yawrate_, max_yawrate_);

      const auto min_thrust = max(min_thrust_, z_rotors_.minThrustSum(battery_->voltage));
      const auto max_thrust = min(max_thrust_, z_rotors_.maxThrustSum(battery_->voltage));
      rpydt->thrust = remap(rcin->thrust, 0., 1., min_thrust, max_thrust);

      // コマンドを発行
      rpydt_pub_.publish(rpydt);
      break;
    }

    default:
    {
      rosthrow(name_, "Invalid state: " << stage_);
    }
  }
}
}  // namespace tobas_rc_teleop

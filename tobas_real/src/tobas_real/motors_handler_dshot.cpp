#include <dh_std_tools/math.hpp>
#include <dh_std_tools/algorithm.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/exception.hpp>
#include <dh_ros_tools/rate.hpp>

#include "../../include/tobas_real/motors_handler_dshot.hpp"
#include "../../include/tobas_real/constants.hpp"

using namespace std;
using namespace dh_std;

namespace tobas_real
{
MotorsHandler_DSHOT::MotorsHandler_DSHOT() : super(), cmd_received_(false), dshot_(DSHOT::DSHOT_600)
{
  if (getuid())
  {
    throw dh_ros::RuntimeError("Not root.");
  }

  getRosParams();
  drone_.loadFromParam(ns_);

  for (const auto& rotor_config : drone_.rotorConfigs())
  {
    dshot_.initialize(rotor_config.pin);
  }

  registerPublishers();
  registerSubscribers();
  createTimers();
}

void MotorsHandler_DSHOT::run()
{
  dh_ros::Rate rate(update_rate_);

  while (ros::ok())
  {
    if (!cmd_received_)
    {
      ros::spinOnce();
      rate.sleep();
      continue;
    }

    for (int i = 0; i < drone_.numRotors(); ++i)
    {
      const auto& rotor_config = drone_.rotorConfig(i);
      const double max_speed = rpmToRadPerSec(drone_.maxRotSpeed(i));

      // 指令速度を決定
      double cmd_speed = cmd_speeds_[i];
      if (cmd_speed < 0.)
      {
        dh_ros::rosErrorThrottle(
          kInfoPeriod, "Rotor speed must be semi-positive: " + to_string(cmd_speed) + " < 0");
        cmd_speed = 0.;
      }
      else if (cmd_speed > max_speed)
      {
        dh_ros::rosErrorThrottle(
          kInfoPeriod, "Commanded rotor speed is too large: " + to_string(cmd_speed) + " > "
                         + to_string(max_speed));
        cmd_speed = max_speed;
      }

      // スロットルに変換して指令
      const uint32_t throttle = remap<double>(cmd_speed, 0., max_speed, 48, (1 << 11) - 1);
      dshot_.setSignal(rotor_config.pin, throttle);
    }

    ros::spinOnce();
    rate.sleep();
  }
}

void MotorsHandler_DSHOT::getRosParams()
{
  dh_ros::getParam("~update_rate", update_rate_, kDefaultUpdateRate);
}

void MotorsHandler_DSHOT::registerPublishers()
{
}

void MotorsHandler_DSHOT::registerSubscribers()
{
  rotor_vels_sub_ =
    nh_.subscribe("command/motor_speed", 1, &MotorsHandler_DSHOT::rotorSpeedsCb, this);
}

void MotorsHandler_DSHOT::createTimers()
{
}

void MotorsHandler_DSHOT::rotorSpeedsCb(const tobas_msgs::RotorSpeeds& rotor_speeds)
{
  const auto& speeds = rotor_speeds.speeds;

  // Check array size
  if (speeds.size() != drone_.numRotors())
  {
    dh_ros::rosErrorThrottle(
      kInfoPeriod,
      "Size mismatch: " + to_string(speeds.size()) + " != " + to_string(drone_.numRotors()));
    return;
  }

  // Check delay
  const double delay = (ros::Time::now() - rotor_speeds.header.stamp).toSec();
  if (delay > kCheckDelayThreshold)
  {
    dh_ros::rosWarnThrottle(
      kInfoPeriod, "The delay from sensors to the motor command is " + to_string(delay)
                     + " seconds, which is too large.");
  }
  else if (delay < 0.)
  {
    dh_ros::rosErrorThrottle(
      kInfoPeriod, "The timestamp of the motor command precedes the current time.");
  }

  if (!cmd_received_)
  {
    cmd_received_ = true;
  }

  cmd_speeds_ = speeds;
}

void MotorsHandler_DSHOT::checkTopicsTimerCb(const ros::TimerEvent& event)
{
}
}  // namespace tobas_real

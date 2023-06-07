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
MotorsHandler_DSHOT::MotorsHandler_DSHOT()
  : super(),
    dshot_(DSHOT::DSHOT_600),
    is_initialized_(false),
    rot_speeds_received_(false),
    battery_received_(false),
    check_topics_timer_(
      nh_,
      kCheckTopicsTimerPeriod,
      &MotorsHandler_DSHOT::checkTopicsTimerCb,
      this)
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
}

void MotorsHandler_DSHOT::run()
{
  dh_ros::Rate rate(update_rate_);

  while (ros::ok())
  {
    if (!is_initialized_)
    {
      if (isReady())
      {
        check_topics_timer_.stop();
        is_initialized_ = true;
      }
      ros::spinOnce();
      rate.sleep();
      continue;
    }

    for (int i = 0; i < drone_.numRotors(); ++i)
    {
      const auto& rotor_config = drone_.rotorConfig(i);
      const auto max_speed = drone_.maxRotSpeed(i, battery_.voltage);

      // 指令速度を決定
      auto cmd_speed = cmd_speeds_[i];
      if (cmd_speed < 0.)
      {
        rosErrorThrottle(kInfoPeriod, "Rotor speed must be semi-positive: " << cmd_speed << " < 0");
        cmd_speed = 0.;
      }
      else if (cmd_speed > max_speed)
      {
        rosErrorThrottle(
          kInfoPeriod, "Commanded rotor speed is too large: " << cmd_speed << " > " << max_speed);
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
  rotor_speeds_sub_ =
    nh_.subscribe("command/motor_speed", 1, &MotorsHandler_DSHOT::rotorSpeedsCb, this);
  battery_sub_ = nh_.subscribe("battery", 1, &MotorsHandler_DSHOT::batteryCb, this);
}

bool MotorsHandler_DSHOT::isReady()
{
  return rot_speeds_received_ && battery_received_;
}

void MotorsHandler_DSHOT::rotorSpeedsCb(const tobas_msgs::RotorSpeeds& rotor_speeds)
{
  if (!rot_speeds_received_)
  {
    rot_speeds_received_ = true;
  }

  const auto& speeds = rotor_speeds.speeds;

  // Check array size
  if (speeds.size() != drone_.numRotors())
  {
    rosErrorThrottle(
      kInfoPeriod, "Size mismatch: " << speeds.size() << " != " << drone_.numRotors());
    return;
  }

  // Check delay
  const auto delay = (ros::Time::now() - rotor_speeds.header.stamp).toSec();
  if (delay > kCheckDelayThreshold)
  {
    rosWarnThrottle(
      kInfoPeriod, "The delay from sensors to the motor command is "
                     << delay << " seconds, which is too large.");
  }
  else if (delay < 0.)
  {
    rosErrorThrottle(kInfoPeriod, "The timestamp of the motor command precedes the current time.");
  }

  cmd_speeds_ = speeds;
}

void MotorsHandler_DSHOT::batteryCb(const tobas_msgs::Battery& battery)
{
  if (!battery_received_)
  {
    battery_received_ = true;
  }

  battery_ = battery;
}

void MotorsHandler_DSHOT::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (!rot_speeds_received_)
  {
    rosWarn("Motor command is not received yet.");
  }

  if (!battery_received_)
  {
    rosWarn("Battery state is not received yet.");
  }
}
}  // namespace tobas_real

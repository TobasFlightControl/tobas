#include <dh_std_tools/math.hpp>
#include <dh_std_tools/algorithm.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/exception.hpp>

#include "../../include/tobas_real/motors_handler_pwm.hpp"
#include "../../include/tobas_real/constants.hpp"

using namespace std;
using namespace dh_std;

namespace tobas_real
{
MotorsHandler_PWM::MotorsHandler_PWM()
  : super(),
    is_initialized_(false),
    rot_speeds_received_(false),
    battery_received_(false),
    check_topics_timer_(nh_, kCheckTopicsTimerPeriod, &MotorsHandler_PWM::checkTopicsTimerCb, this)
{
  if (getuid())
  {
    throw dh_ros::RuntimeError("Not root.");
  }

  getRosParams();
  drone_.loadFromParam(ns_);

  for (const auto& rotor_config : drone_.rotorConfigs())
  {
    const uint32_t& pin = rotor_config.pin;
    uint32_t channel = getChannel(pin);

    if (!pwm_.initialize(channel))
    {
      throw dh_ros::RuntimeError("Failed to initialize RC output for PIN" + to_string(pin) + ".");
    }

    if (!pwm_.set_frequency(channel, kPwmFrequency))
    {
      throw dh_ros::RuntimeError("Failed to set frequency for PIN" + to_string(pin) + ".");
    }

    if (!pwm_.enable(channel))
    {
      throw dh_ros::RuntimeError("RC output for PIN" + to_string(pin) + " is disabled.");
    }

    rosInfo("PWM output for PIN" << pin << " is ready.");
    ros::Duration(0.2).sleep();  // 連続して設定を行うと失敗するため間隔をあける
  }

  registerPublishers();
  registerSubscribers();
}

void MotorsHandler_PWM::getRosParams()
{
}

void MotorsHandler_PWM::registerPublishers()
{
}

void MotorsHandler_PWM::registerSubscribers()
{
  rotor_speeds_sub_ =
    nh_.subscribe("command/motor_speed", 1, &MotorsHandler_PWM::rotorSpeedsCb, this);
  battery_sub_ = nh_.subscribe("battery", 1, &MotorsHandler_PWM::batteryCb, this);
}

bool MotorsHandler_PWM::isReady()
{
  return rot_speeds_received_ && battery_received_;
}

uint32_t MotorsHandler_PWM::getChannel(uint32_t pin)
{
  return pin - 1;
}

void MotorsHandler_PWM::rotorSpeedsCb(const tobas_msgs::RotorSpeeds& rotor_speeds)
{
  if (!rot_speeds_received_)
  {
    rot_speeds_received_ = true;
  }

  const auto& cmd_speeds = rotor_speeds.speeds;

  // Check array size
  if (cmd_speeds.size() != drone_.numRotors())
  {
    rosErrorThrottle(
      kInfoPeriod, "Size mismatch: " << cmd_speeds.size() << " != " << drone_.numRotors());
    return;
  }

  // Initialize
  if (!is_initialized_)
  {
    if (isReady())
    {
      check_topics_timer_.stop();
      is_initialized_ = true;
    }
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

  for (int i = 0; i < drone_.numRotors(); ++i)
  {
    const auto& rotor_config = drone_.rotorConfig(i);
    const auto max_speed = drone_.maxRotSpeed(i, battery_.voltage);

    // 指令速度を決定
    auto cmd_speed = cmd_speeds[i];
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

    // パルス幅に変換して指令
    const auto& pin = rotor_config.pin;
    const auto period = remap(cmd_speed, 0., max_speed, kPwmMin, kPwmMax);
    if (!pwm_.set_duty_cycle(getChannel(pin), period))
    {
      throw dh_ros::RuntimeError("Failed to set PWM duty cycle for PIN" + to_string(pin) + ".");
    }
  }
}

void MotorsHandler_PWM::batteryCb(const tobas_msgs::Battery& battery)
{
  if (!battery_received_)
  {
    battery_received_ = true;
  }

  battery_ = battery;
}

void MotorsHandler_PWM::checkTopicsTimerCb(const ros::TimerEvent&)
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

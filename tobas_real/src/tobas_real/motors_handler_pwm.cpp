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
MotorsHandler_PWM::MotorsHandler_PWM() : super()
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

    if (!pwm_.set_frequency(channel, rotor_config.pwm.frequency))
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
  createTimers();
}

void MotorsHandler_PWM::getRosParams()
{
}

void MotorsHandler_PWM::registerPublishers()
{
}

void MotorsHandler_PWM::registerSubscribers()
{
  rotor_vels_sub_ =
    nh_.subscribe("command/motor_speed", 1, &MotorsHandler_PWM::rotorSpeedsCb, this);
}

void MotorsHandler_PWM::createTimers()
{
}

uint32_t MotorsHandler_PWM::getChannel(uint32_t pin)
{
  return pin - 1;
}

void MotorsHandler_PWM::rotorSpeedsCb(const tobas_msgs::RotorSpeeds& rotor_speeds)
{
  const auto& cmd_speeds = rotor_speeds.speeds;

  // Check array size
  if (cmd_speeds.size() != drone_.numRotors())
  {
    rosErrorThrottle(
      kInfoPeriod, "Size mismatch: " << cmd_speeds.size() << " != " << drone_.numRotors());
    return;
  }

  // Check delay
  const double delay = (ros::Time::now() - rotor_speeds.header.stamp).toSec();
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
    const double max_speed = drone_.maxRotSpeed(i);

    // 指令速度を決定
    double cmd_speed = cmd_speeds[i];
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
    const auto& pwm = rotor_config.pwm;
    const double period =
      remap(cmd_speed, 0., max_speed, pwm.pulse_width_range.lower, pwm.pulse_width_range.upper);
    if (!pwm_.set_duty_cycle(getChannel(pin), period))
    {
      throw dh_ros::RuntimeError("Failed to set PWM duty cycle for PIN" + to_string(pin) + ".");
    }
  }
}

void MotorsHandler_PWM::checkTopicsTimerCb(const ros::TimerEvent& event)
{
}
}  // namespace tobas_real

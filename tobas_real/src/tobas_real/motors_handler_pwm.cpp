#include <dh_std_tools/math.hpp>
#include <dh_std_tools/algorithm.hpp>
#include <dh_std_tools/vector.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/exception.hpp>
#include <dh_ros_tools/rate.hpp>

#include "../../include/tobas_real/motors_handler_pwm.hpp"
#include "../../include/tobas_real/common.hpp"

using namespace std;
using namespace dh_std;

namespace tobas_real
{
MotorsHandler_PWM::MotorsHandler_PWM()
  : super(),
    last_cmd_time_(0.),
    is_activated_(false),
    is_initialized_(false),
    battery_received_(false),
    check_topics_timer_(nh_, kCheckTopicsTimerPeriod, &MotorsHandler_PWM::checkTopicsTimerCb, this)
{
  if (getuid())
  {
    rosthrow("Not root.");
  }

  getRosParams();
  drone_.loadFromParam(ns_);

  // PWMドライバのセットアップ
  for (const auto& rotor_config : drone_.rotorConfigs())
  {
    const auto channel = channelFromPin(rotor_config.pin);
    setupRCOutput(pwm_, channel);
  }

  // コマンドの初期値はDisarm
  pwm_periods_.resize(drone_.numRotors(), kPwmDisarm);

  registerPublishers();
  registerSubscribers();
}

void MotorsHandler_PWM::run()
{
  rosInfo("Send disarm command for " << kDisarmDuration << " seconds.");
  sendDisarm();
  rosInfo("Disarming finished. The motors are ready to rotate.");

  dh_ros::Rate rate(kControlRate);
  while (ros::ok())
  {
    // Check elapsed time after last command
    const auto time_after_last_cmd = (ros::Time::now() - last_cmd_time_).toSec();
    if (is_activated_ && time_after_last_cmd > kAutoStopTimeThreshold)
    {
      dh_std::fill(pwm_periods_, kPwmDisarm);
      is_activated_ = false;
      rosInfo(
        "The rotors are automatically stopped because "
        << kAutoStopTimeThreshold << " seconds have elapsed since the last command.");
    }

    // Set PWM duty cycles
    for (uint32_t rotor_idx = 0; rotor_idx < drone_.numRotors(); ++rotor_idx)
    {
      const auto& rotor_config = drone_.rotorConfig(rotor_idx);
      const auto& pin = rotor_config.pin;
      if (!pwm_.set_duty_cycle(channelFromPin(pin), pwm_periods_[rotor_idx]))
      {
        rosFatal("Failed to set PWM duty cycle on PIN" << pin << ".");
      }
    }

    ros::spinOnce();
    rate.sleep();
  }
}

void MotorsHandler_PWM::getRosParams()
{
}

void MotorsHandler_PWM::registerPublishers()
{
}

void MotorsHandler_PWM::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &MotorsHandler_PWM::eventCb, this);
  rotor_speeds_sub_ =
    nh_.subscribe("command/motor_speed", 1, &MotorsHandler_PWM::rotorSpeedsCb, this);
  battery_sub_ = nh_.subscribe("battery", 1, &MotorsHandler_PWM::batteryCb, this);
}

bool MotorsHandler_PWM::isReady()
{
  return battery_received_;
}

void MotorsHandler_PWM::sendDisarm()
{
  const ros::Time start_time = ros::Time::now();
  while ((ros::Time::now() - start_time).toSec() < kDisarmDuration)
  {
    for (const auto& rotor_config : drone_.rotorConfigs())
    {
      const auto& pin = rotor_config.pin;
      if (!pwm_.set_duty_cycle(channelFromPin(pin), kPwmDisarm))
      {
        rosFatal("Failed to set PWM duty cycle on PIN " << pin << ".");
      }
    }
    ros::Duration(kDisarmInterval).sleep();
  }
}

void MotorsHandler_PWM::eventCb(const tobas_msgs::Event& event)
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

void MotorsHandler_PWM::rotorSpeedsCb(const tobas_msgs::RotorSpeeds& rotor_speeds)
{
  if (!is_initialized_)
  {
    if (isReady())
    {
      check_topics_timer_.stop();
      is_initialized_ = true;
    }
    else
    {
      rosError("The rotors cannot be rotated because some topics have not been received yet.");
      return;
    }
  }

  // Check array size
  if (rotor_speeds.speeds.size() != drone_.numRotors())
  {
    rosErrorThrottle(
      kErrorPeriod,
      "Size mismatch: " << rotor_speeds.speeds.size() << " != " << drone_.numRotors());
    return;
  }

  const auto cur_time = ros::Time::now();

  // Check delay
  const auto delay = (cur_time - rotor_speeds.header.stamp).toSec();
  if (delay > kCheckDelayThreshold)
  {
    rosWarnThrottle(
      kErrorPeriod, "The delay from sensors to the motor command is "
                      << delay << ", which exceeds the threshold " << kCheckDelayThreshold);
  }
  else if (delay < 0.)
  {
    rosErrorThrottle(kErrorPeriod, "The timestamp of the motor command precedes the current time.");
  }

  // Update PWM periods
  for (uint32_t rotor_idx = 0; rotor_idx < drone_.numRotors(); ++rotor_idx)
  {
    const auto& rotor_config = drone_.rotorConfig(rotor_idx);
    const auto& pin = rotor_config.pin;
    const auto max_speed = drone_.maxRotSpeed(rotor_idx, battery_.voltage);

    // 指令速度を決定
    auto cmd_speed = rotor_speeds.speeds[rotor_idx];
    if (cmd_speed < 0.)
    {
      rosErrorThrottle(
        kErrorPeriod,
        "Negative rotor speed is commanded on PIN" << pin << ": " << cmd_speed << " [rad/s]");
      cmd_speed = 0.;
    }
    else if (cmd_speed > max_speed)
    {
      rosErrorThrottle(
        kErrorPeriod, "Commanded rotor speed on PIN" << pin << " exceeds the limit: " << cmd_speed
                                                     << " > " << max_speed << " [rad/s]");
      cmd_speed = max_speed;
    }

    // パルス幅に変換
    pwm_periods_[rotor_idx] = remap(cmd_speed, 0., max_speed, kPwmMin, kPwmMax);
  }

  // Update last commanded time
  last_cmd_time_ = cur_time;

  // Now the motor is activated
  is_activated_ = true;
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
  if (!battery_received_)
  {
    rosWarn("Battery state is not received yet.");
  }
}
}  // namespace tobas_real

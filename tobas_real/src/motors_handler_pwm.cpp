#include <dh_std_tools/math.hpp>
#include <dh_std_tools/algorithm.hpp>
#include <dh_std_tools/vector.hpp>
#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/exception.hpp>
#include <dh_ros_tools/rate.hpp>

#include "../include/tobas_real/motors_handler_pwm.hpp"
#include "../include/tobas_real/common.hpp"

using namespace std;
using namespace dh_std;

namespace tobas_real
{
MotorsHandler::MotorsHandler(ros::NodeHandle nh, ros::NodeHandle pnh)
  : super(nh, pnh), is_activated_(false), battery_received_(false)
{
  if (getuid())
  {
    rosthrow("Not root.");
  }

  getRosParams();
  drone_.loadFromParam(nh_);

  // PWMドライバのセットアップ
  for (const auto& rotor_config : drone_.rotorConfigs())
  {
    const auto channel = channelFromPin(rotor_config.pin);
    setupRCOutput(pwm_, channel);
  }

  // Send disarm command
  rosInfo("Sending disarm command for " << kDisarmDuration << " seconds.");
  sendDisarm();
  rosInfo("Disarming finished. The motors are ready to rotate.");

  registerPublishers();
  registerSubscribers();

  check_interval_timer_ = nh_.createTimer(
    ros::Duration(1 / kCheckIntervalRate), &MotorsHandler::checkIntervalTimerCb, this);
}

void MotorsHandler::getRosParams()
{
}

void MotorsHandler::registerPublishers()
{
}

void MotorsHandler::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &MotorsHandler::eventCb, this);
  rotor_speeds_sub_ = nh_.subscribe("command/motor_speed", 1, &MotorsHandler::rotorSpeedsCb, this);
  battery_sub_ = nh_.subscribe("battery", 1, &MotorsHandler::batteryCb, this);
}

bool MotorsHandler::isReady()
{
  return battery_received_;
}

void MotorsHandler::sendDisarm()
{
  const ros::Time start_time = ros::Time::now();
  while ((ros::Time::now() - start_time).toSec() < kDisarmDuration)
  {
    setPeriodOnAllChannels(kPwmDisarm);
    ros::Duration(kDisarmInterval).sleep();
  }
}

void MotorsHandler::setPeriodOnAllChannels(double period)
{
  for (const auto& rotor_config : drone_.rotorConfigs())
  {
    const auto& pin = rotor_config.pin;
    if (!pwm_.set_duty_cycle(channelFromPin(pin), period))
    {
      rosFatal("Failed to set PWM duty cycle on PIN " << pin << ".");
      // TODO: Request shutdown
    }
  }
}

void MotorsHandler::eventCb(const tobas_msgs::Event& event)
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

void MotorsHandler::rotorSpeedsCb(const tobas_msgs::RotorSpeeds& rotor_speeds)
{
  if (!battery_received_)
  {
    rosWarn("The rotors cannot be rotated because battery state has not been received yet.");
    return;
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
  // rosInfo("The delay from IMU to the motor command: " << delay << "[s]");
  if (delay > kCheckDelayThreshold)
  {
    rosWarnThrottle(
      kErrorPeriod, "The delay from IMU to the motor command is "
                      << delay << ", which exceeds the threshold " << kCheckDelayThreshold);
  }
  else if (delay < 0.)
  {
    rosErrorThrottle(kErrorPeriod, "The timestamp of the motor command precedes the current time.");
  }

  // Update PWM periods
  for (uint32_t rotor_idx = 0; rotor_idx < drone_.numRotors(); ++rotor_idx)
  {
    // スロットルを決定
    // 電圧とスロットルの関係は線形だが，電圧と回転数の関係は非線形であることに注意
    const auto& pin = drone_.rotorConfig(rotor_idx).pin;
    const auto cmd_voltage = drone_.voltageFromRotSpeed(rotor_idx, rotor_speeds.speeds[rotor_idx]);
    auto tar_throttle = cmd_voltage / battery_.voltage;  // [0, 1]
    if (tar_throttle < tobas::kMotorSpinArm - kThrottleMargin)
    {
      rosErrorThrottle(
        kErrorPeriod, "Target throttle on PIN" << pin << " is too low: " << tar_throttle << " < "
                                               << tobas::kMotorSpinArm);
      tar_throttle = tobas::kMotorSpinArm;
    }
    if (tar_throttle > 1. + kThrottleMargin)
    {
      rosErrorThrottle(
        kErrorPeriod,
        "Target throttle on PIN" << pin << " is too high: " << tar_throttle << " > 1");
      tar_throttle = 1.;
    }

    // スロットルをパルス幅に変換
    const auto pwm_period = remap(tar_throttle, 0., 1., kPwmMin, kPwmMax);

    // Set PWM duty cycle
    if (!pwm_.set_duty_cycle(channelFromPin(pin), pwm_period))
    {
      rosFatal("Failed to set PWM duty cycle on PIN" << pin << ".");
      // TODO: Request shutdown
    }
  }

  // Update last commanded time
  last_cmd_time_ = cur_time;

  // Now the motors are activated
  is_activated_ = true;
}

void MotorsHandler::batteryCb(const tobas_msgs::Battery& battery)
{
  if (!battery_received_)
  {
    battery_received_ = true;
  }

  battery_ = battery;
}

void MotorsHandler::checkIntervalTimerCb(const ros::TimerEvent& event)
{
  if (!is_activated_)
  {
    return;
  }

  // Check elapsed time after last command
  const auto time_after_last_cmd = (event.current_real - last_cmd_time_).toSec();
  if (time_after_last_cmd > kAutoStopTimeThreshold)
  {
    setPeriodOnAllChannels(kPwmArm);
    is_activated_ = false;
    rosInfo(
      "The rotors are automatically slowed down because "
      << kAutoStopTimeThreshold << " seconds have elapsed since the last command.");
  }
}
}  // namespace tobas_real

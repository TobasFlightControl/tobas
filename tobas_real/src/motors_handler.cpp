#include <dh_std_tools/math.hpp>
#include <dh_std_tools/algorithm.hpp>
#include <dh_std_tools/vector.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include "../include/tobas_real/motors_handler.hpp"
#include "../include/tobas_real/common.hpp"

using namespace std;
using namespace dh_std;

namespace tobas_real
{
MotorsHandler::MotorsHandler(ros::NodeHandle nh, ros::NodeHandle pnh, string name)
  : super(nh, pnh, name)
{
  if (getuid())
  {
    rosthrow(name_, "Not root.");
  }

  getRosParams();
  drone_.loadFromParam(nh_);

  latency_filter_.initialize(kCheckLatencyTimeConst, 0.);

  // PWMドライバのセットアップ
  for (const auto& rotor_config : drone_.rotorConfigs())
  {
    const auto channel = channelFromPin(rotor_config.pin);
    setupRCOutput(pwm_, channel);
  }

  // Send disarm command
  rosInfo(name_, "Sending disarm command for " << kDisarmDuration << " seconds.");
  sendDisarm();
  rosInfo(name_, "Disarming finished. The motors are ready to rotate.");

  registerPublishers();
  registerSubscribers();

  check_interval_timer_ =
    nh_.createTimer(kCheckIntervalRate, &MotorsHandler::checkIntervalTimerCb, this);
}

void MotorsHandler::getRosParams()
{
}

void MotorsHandler::registerPublishers()
{
}

void MotorsHandler::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &MotorsHandler::eventCb, this, tcpNoDelay());
  rotor_speeds_sub_ =
    nh_.subscribe("command/motor_speed", 1, &MotorsHandler::rotorSpeedsCb, this, tcpNoDelay());
  battery_sub_ = nh_.subscribe("battery", 1, &MotorsHandler::batteryCb, this, tcpNoDelay());
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

void MotorsHandler::setPeriodOnAllChannels(const double& period)
{
  for (const auto& rotor_config : drone_.rotorConfigs())
  {
    const auto& pin = rotor_config.pin;
    if (!pwm_.setDutyCycle(channelFromPin(pin), period))
    {
      rosFatal(name_, "Failed to set PWM duty cycle on PIN " << pin << ".");
      // TODO: Request shutdown
    }
  }
}

void MotorsHandler::eventCb(const tobas_msgs::EventConstPtr& event)
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

void MotorsHandler::rotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& rotor_speeds)
{
  if (battery_ == nullptr)
  {
    rosWarn(name_, "The rotors cannot be rotated because battery state has not been received yet.");
    return;
  }

  // Check array size
  if (rotor_speeds->speeds.size() != drone_.numRotors())
  {
    rosErrorThrottle(
      kErrorPeriod, name_,
      "Size mismatch: " << rotor_speeds->speeds.size() << " != " << drone_.numRotors());
    return;
  }

  // Get current time
  const auto cur_time = ros::Time::now();

  // Update PWM periods
  for (uint32_t rotor_idx = 0; rotor_idx < drone_.numRotors(); ++rotor_idx)
  {
    // スロットルを決定
    // 電圧とスロットルの関係は線形だが，電圧と回転数の関係は非線形であることに注意
    const auto& pin = drone_.rotorConfig(rotor_idx).pin;
    const auto cmd_voltage = drone_.voltageFromRotSpeed(rotor_idx, rotor_speeds->speeds[rotor_idx]);
    auto tar_throttle = cmd_voltage / battery_->voltage;  // [0, 1]
    if (tar_throttle < tobas::kMotorSpinArm - kThrottleMargin)
    {
      rosErrorThrottle(
        kErrorPeriod, name_,
        "Target throttle on PIN" << pin << " is too low: " << tar_throttle << " < "
                                 << tobas::kMotorSpinArm);
      tar_throttle = tobas::kMotorSpinArm;
    }
    if (tar_throttle > 1. + kThrottleMargin)
    {
      rosErrorThrottle(
        kErrorPeriod, name_,
        "Target throttle on PIN" << pin << " is too high: " << tar_throttle << " > 1");
      tar_throttle = 1.;
    }

    // スロットルをパルス幅に変換
    const auto pwm_period = remap(tar_throttle, 0., 1., kPwmMin, kPwmMax);

    // Set PWM duty cycle
    if (!pwm_.setDutyCycle(channelFromPin(pin), pwm_period))
    {
      rosFatal(name_, "Failed to set PWM duty cycle on PIN" << pin << ".");
      // TODO: Request shutdown
    }
  }

  // Check latency: 多少の外れ値を許容するため，LPFを通したレイテンシで評価
  // TODO: LPFを通したレイテンシで評価するのは妥当なのか．本当は最悪時間を見るべきでは？
  if (is_activated_)
  {
    const auto latency = (cur_time - rotor_speeds->header.stamp).toSec();
    if (latency < 0.)
    {
      rosErrorThrottle(
        kErrorPeriod, name_, "The timestamp of the motor command precedes the current time.");
      return;
    }

    const auto dt = (cur_time - last_cmd_time_).toSec();
    latency_filter_.update(latency, dt);
    const auto& filtered_latency = latency_filter_.getState();
    if (filtered_latency > kCheckLatencyThreshold)
    {
      rosWarn(
        name_, "The time averaged latency from IMU to the motor command is "
                 << filtered_latency << ", which exceeds the threshold " << kCheckLatencyThreshold);
    }

    // cout << "Raw Latency[s]     : " << latency << endl;
    // cout << "Filtered latency[s]: " << filtered_latency << endl;
  }

  // Update last commanded time
  last_cmd_time_ = cur_time;

  // Now the motors are activated
  is_activated_ = true;
}

void MotorsHandler::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  battery_ = battery;
}

void MotorsHandler::checkIntervalTimerCb(const ros::TimerEvent& event)
{
  const auto time_after_last_cmd = (event.current_real - last_cmd_time_).toSec();
  if (time_after_last_cmd > kAutoStopTimeThreshold)
  {
    setPeriodOnAllChannels(kPwmArm);
    if (is_activated_)
    {
      latency_filter_.initialize(kCheckLatencyTimeConst, 0.);
      is_activated_ = false;
      rosInfo(
        name_, "The rotors are automatically slowed down because "
                 << kAutoStopTimeThreshold << " seconds have elapsed since the last command.");
    }
  }
}
}  // namespace tobas_real

#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/algorithm.hpp>
#include <tobas_std_tools/vector.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>

#include "../include/tobas_real/motors_handler.hpp"
#include "../include/tobas_real/common.hpp"

#define SETUP_PWM_INTERVAL 100000  // [us]

using namespace std;
using namespace tobas_std;

namespace tobas_real
{
MotorsHandler::MotorsHandler(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name)
{
  getRosParams();
  drone_.loadFromParam(nh_);

  latency_filter_.initialize(kCheckLatencyTimeConst, 0.);

  // PWMのセットアップを開始
  setup_pwm_timer_ = nh_.createTimer(kSetupPwmTimerRate, &self::setupPwmTimerCb, this);
}

void MotorsHandler::getRosParams()
{
}

void MotorsHandler::registerPublishers()
{
  rotor_speeds_pub_ = nh_.advertise<tobas_msgs::RotorSpeeds>(tobas::kRotorSpeedsTopic, 1);
}

void MotorsHandler::registerSubscribers()
{
  super::registerSubscribers();

  throttles_sub_ =
    nh_.subscribe(tobas::kThrottlesCmdTopic, 1, &self::throttlesCmdCb, this, tcpNoDelay());
  battery_sub_ = nh_.subscribe(tobas::kBatteryTopic, 1, &self::batteryCb, this, tcpNoDelay());
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
    case tobas_msgs::Event::STOP:
      nh_.shutdown();
      check_interval_timer_.stop();
      break;
    default:
      break;
  }
}

void MotorsHandler::throttlesCmdCb(const tobas_msgs::ThrottlesConstPtr& throttles)
{
  if (battery_ == nullptr)
  {
    rosWarn(name_, "The rotors cannot be rotated because battery state has not been received yet.");
    return;
  }

  // Check array size
  if (throttles->data.size() != drone_.numRotors())
  {
    rosError(name_, "Size mismatch: " << throttles->data.size() << " != " << drone_.numRotors());
    return;
  }

  // Get current time
  const auto cur_time = ros::Time::now();

  // Create real rotating speeds
  const auto real_speeds = boost::make_shared<tobas_msgs::RotorSpeeds>();
  real_speeds->header.stamp = cur_time;
  real_speeds->speeds.resize(drone_.numRotors());

  // Update PWM periods
  for (size_t rotor_idx = 0; rotor_idx < drone_.numRotors(); ++rotor_idx)
  {
    const auto& pin = drone_.rotorConfig(rotor_idx).pin;

    // スロットルを決定
    auto tar_throttle = throttles->data[rotor_idx];
    if (tar_throttle < tobas::kArmThrottle - kThrottleMargin)
    {
      rosWarnThrottle(
        kWarnPeriod, name_,
        "Target throttle on PIN" << pin << " is too low: " << tar_throttle << " < "
                                 << tobas::kArmThrottle);
      tar_throttle = tobas::kArmThrottle;
    }
    if (tar_throttle > tobas::kMaxThrottle - kThrottleMargin)
    {
      rosWarnThrottle(kWarnPeriod, name_, "Full throttle is commanded to PIN " << pin);
      tar_throttle = tobas::kMaxThrottle;
    }

    // スロットルをパルス幅に変換
    const auto pwm_period =
      remap<double>(tar_throttle, tobas::kMinThrottle, tobas::kMaxThrottle, kPwmMin, kPwmMax);

    // Set PWM duty cycle
    if (!pwm_.setDutyCycle(channelFromPin(pin), pwm_period))
    {
      rosFatal(name_, "Failed to set PWM duty cycle on PIN" << pin << ".");
      // TODO: Request shutdown
    }

    // 実際に印加される電圧から回転数を計算．モータ遅延は無視している．
    // TODO: エンコーダを用いて真の回転数が取得できる場合に対応
    const auto real_voltage = battery_->voltage * tar_throttle;
    real_speeds->speeds[rotor_idx] = drone_.rotSpeedFromVoltage(rotor_idx, real_voltage);
  }

  // Publish real rotating speeds
  rotor_speeds_pub_.publish(real_speeds);

  // Check latency: 多少の外れ値を許容するため，LPFを通したレイテンシで評価
  // TODO: LPFを通したレイテンシで評価するのは妥当なのか．本当は最悪時間を見るべきでは？
  if (is_activated_)
  {
    const auto latency = (cur_time - throttles->header.stamp).toSec();
    if (latency < 0)
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

void MotorsHandler::setupPwmTimerCb(const ros::TimerEvent& event)
{
  for (const auto& rotor_config : drone_.rotorConfigs())
  {
    const auto channel = channelFromPin(rotor_config.pin);
    if (!pwm_.initialize(channel))
    {
      rosWarn(name_, "Failed to initialize RC output on CH" + to_string(channel) + ". Retrying...");
      return;
    }
    if (!pwm_.setFrequency(channel, kPwmFrequency))
    {
      rosWarn(name_, "Failed to set PWM frequency on CH" + to_string(channel) + ". Retrying...");
      return;
    }
    if (!pwm_.enable(channel))
    {
      rosWarn(name_, "Failed to enable RC output on CH" + to_string(channel) + ". Retrying...");
      return;
    }

    usleep(SETUP_PWM_INTERVAL);
  }

  // Disarmを開始
  setup_pwm_timer_.stop();
  disarm_timer_ = nh_.createTimer(kDisarmTimerRate, &self::disarmTimerCb, this);

  disarm_start_time_ = event.current_real;
  rosInfo(name_, "Sending disarm command for " << kDisarmDuration << " seconds.");
}

void MotorsHandler::disarmTimerCb(const ros::TimerEvent& event)
{
  setPeriodOnAllChannels(kPwmDisarm);

  if ((event.current_real - disarm_start_time_).toSec() > kDisarmDuration)
  {
    registerPublishers();
    registerSubscribers();

    // コマンドのインターバルチェックを開始
    disarm_timer_.stop();
    check_interval_timer_ =
      nh_.createTimer(kCheckIntervalTimerRate, &self::checkIntervalTimerCb, this);

    rosInfo(name_, "Disarming finished. The motors are ready to rotate.");
  }
}

void MotorsHandler::checkIntervalTimerCb(const ros::TimerEvent& event)
{
  const auto time_after_last_cmd = (event.current_real - last_cmd_time_).toSec();
  if (time_after_last_cmd > tobas::kAutoResetTimeThreshold)
  {
    setPeriodOnAllChannels(kPwmMin);
    if (is_activated_)
    {
      latency_filter_.initialize(kCheckLatencyTimeConst, 0.);
      is_activated_ = false;
      rosWarn(
        name_, "The speeds of all rotors are automatically stopped because "
                 << tobas::kAutoResetTimeThreshold
                 << " seconds have elapsed since the last command.");
    }

    // Publish arming speeds
    if (battery_ != nullptr)
    {
      const auto real_speeds = boost::make_shared<tobas_msgs::RotorSpeeds>();
      real_speeds->header.stamp = event.current_real;
      real_speeds->speeds.resize(drone_.numRotors());

      const auto real_voltage = battery_->voltage * tobas::kArmThrottle;
      for (size_t rotor_idx = 0; rotor_idx < drone_.numRotors(); ++rotor_idx)
      {
        real_speeds->speeds[rotor_idx] = drone_.rotSpeedFromVoltage(rotor_idx, real_voltage);
      }

      rotor_speeds_pub_.publish(real_speeds);
    }
  }
}
}  // namespace tobas_real

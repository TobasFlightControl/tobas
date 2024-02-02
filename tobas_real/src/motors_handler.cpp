#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/algorithm.hpp>
#include <tobas_std_tools/vector.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_tools/utils.hpp>
#include <tobas_msgs/PwmArray.h>
#include <tobas_msgs/RotorSpeeds.h>
#include <tobas_msgs/Latency.h>
#include <tobas_msgs/SetupPwm.h>

#include "../include/tobas_real/motors_handler.hpp"
#include "../include/tobas_real/common.hpp"

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

  registerPublishers();
  registerSubscribers();

  arm_rotors_ss_ = nh_.advertiseService(tobas::kArmRotorsSrv, &self::armRotorsCb, this);
  setup_pwm_sc_ = nh_.serviceClient<tobas_msgs::SetupPwm>(tobas::kSetupPwmSrv);

  setup_pwm_timer_ =
    nh_.createTimer(kSetupPwmTimerRate, &self::setupPwmTimerCb, this, false, false);
  disarm_timer_ = nh_.createTimer(kDisarmTimerRate, &self::disarmTimerCb, this, false, false);
  check_interval_timer_ =
    nh_.createTimer(kCheckIntervalTimerRate, &self::checkIntervalTimerCb, this, false, false);

  // PWMのセットアップを開始
  setup_pwm_timer_.start();
}

void MotorsHandler::getRosParams()
{
  tobas_ros::getParam(
    pnh_, "block_below_arm_speed", block_below_arm_speed_, kDefaultBlockBelowArmSpeed);
}

void MotorsHandler::registerPublishers()
{
  pwms_pub_ = nh_.advertise<tobas_msgs::PwmArray>(tobas::kPwmCmdTopic, 1);
  cur_speeds_pub_ = nh_.advertise<tobas_msgs::RotorSpeeds>(tobas::kRotorSpeedsTopic, 1);
  latency_pub_ = nh_.advertise<tobas_msgs::Latency>(tobas::kLatencyTopic, 1);
}

void MotorsHandler::registerSubscribers()
{
  tar_speeds_sub_ =
    nh_.subscribe(tobas::kRotorSpeedsCmdTopic, 1, &self::rotSpeedsCmdCb, this, tcpNoDelay());
  battery_sub_ = nh_.subscribe(tobas::kBatteryLpfTopic, 1, &self::batteryCb, this, tcpNoDelay());
}

void MotorsHandler::setPeriodOnAllChannels(const double& period)
{
  const auto pwms = boost::make_shared<tobas_msgs::PwmArray>();

  for (const auto& rotor_config : drone_.rotorConfigs())
  {
    pwm_.channel = channelFromPin(rotor_config.pin);
    pwm_.period = period;
    pwms->pwm.push_back(pwm_);
  }

  pwms_pub_.publish(pwms);
}

void MotorsHandler::rotSpeedsCmdCb(const tobas_msgs::RotorSpeedsConstPtr& tar_speeds)
{
  if (battery_ == nullptr)
  {
    rosErrorThrottle(
      kErrorPeriod, name_,
      "The rotors cannot be rotated because battery state has not been received yet.");
    return;
  }

  if (!is_armed_)
  {
    rosErrorThrottle(
      kErrorPeriod, name_, "The rotors cannot be rotated because they are disarmed.");
    return;
  }

  const auto data_size = tar_speeds->speeds.size();
  if (data_size != drone_.numRotors())
  {
    rosError(name_, "Size mismatch: " << data_size << " != " << drone_.numRotors());
    return;
  }

  // Get current time
  const auto cur_time = ros::Time::now();

  // Create ROS messages
  const auto pwms = boost::make_shared<tobas_msgs::PwmArray>();
  const auto real_speeds = boost::make_shared<tobas_msgs::RotorSpeeds>();
  real_speeds->header.stamp = cur_time;
  real_speeds->speeds.resize(data_size);

  // Update PWM periods
  for (size_t rotor_idx = 0; rotor_idx < data_size; ++rotor_idx)
  {
    const auto& pin = drone_.rotorConfig(rotor_idx).pin;
    const auto channel = channelFromPin(pin);

    // 目標回転数を決定
    const auto min_speed =
      block_below_arm_speed_ ? drone_.minRotSpeed(rotor_idx, battery_->voltage) : 0.;
    const auto max_speed = drone_.maxRotSpeed(rotor_idx, battery_->voltage);
    auto tar_speed = tar_speeds->speeds[rotor_idx];
    if (tar_speed < min_speed - tobas::kRotSpeedMargin)
    {
      ROS_WARN_STREAM(
        "Target rotation speed of CH" << rotor_idx << " is too low: " << tar_speed << " < "
                                      << min_speed << " [rad/s]");
      tar_speed = min_speed;
    }
    else if (tar_speed > max_speed + tobas::kRotSpeedMargin)
    {
      ROS_WARN_STREAM(
        "Target rotation speed of CH" << rotor_idx << " is too high: " << tar_speed << " > "
                                      << max_speed << " [rad/s]");
      tar_speed = max_speed;
    }

    // モータの追従遅延やモデル化誤差を無視し，目標回転数をそのまま現在の回転数としてメッセージに格納．
    // TODO: エンコーダを用いて真の回転数が取得できる場合に対応
    real_speeds->speeds[rotor_idx] = tar_speed;

    // PWMコマンドメッセージを作成
    switch (drone_.rotorConfig(rotor_idx).esc_signal_mode)
    {
      case tobas::EscSignalMode::BLHELI_OPEN_LOOP:
      {
        const auto throttle = drone_.throttleFromRotSpeed(rotor_idx, tar_speed, battery_->voltage);
        pwm_.period = remap(throttle, tobas::kMinThrottle, tobas::kMaxThrottle, kPwmMin, kPwmMax);
        break;
      }
      case tobas::EscSignalMode::BLHELI_CLOSED_LOOP_LOW_RANGE:
      {
        const auto tar_erpm = drone_.erpmFromRotSpeed(rotor_idx, tar_speed);
        pwm_.period = remap(tar_erpm, 0., kBLHeliClosedLoopLowRangeMaxERPM, kPwmMin, kPwmMax);
        break;
      }
      case tobas::EscSignalMode::BLHELI_CLOSED_LOOP_MID_RANGE:
      {
        const auto tar_erpm = drone_.erpmFromRotSpeed(rotor_idx, tar_speed);
        pwm_.period = remap(tar_erpm, 0., kBLHeliClosedLoopMidRangeMaxERPM, kPwmMin, kPwmMax);
        break;
      }
      case tobas::EscSignalMode::BLHELI_CLOSED_LOOP_HIGH_RANGE:
      {
        const auto tar_erpm = drone_.erpmFromRotSpeed(rotor_idx, tar_speed);
        pwm_.period = remap(tar_erpm, 0., kBLHeliClosedLoopHighRangeMaxERPM, kPwmMin, kPwmMax);
        break;
      }
      default:
      {
        rosError(name_, "Unknown ESC signal mode of CH" << channel);
        break;
      }
    }
    pwm_.channel = channel;
    pwms->pwm.push_back(pwm_);
  }

  // Publish PWM commands
  pwms_pub_.publish(pwms);

  // Publish real rotation speeds
  cur_speeds_pub_.publish(real_speeds);

  // Check latency: 多少の外れ値を許容するため，LPFを通したレイテンシで評価
  // TODO: LPFを通したレイテンシで評価するのは妥当なのか．本当は最悪時間を見るべきでは？
  if (is_activated_)
  {
    const auto latency = boost::make_shared<tobas_msgs::Latency>();
    latency->data = (cur_time - tar_speeds->header.stamp).toSec();
    latency_pub_.publish(latency);

    if (latency->data < 0)
    {
      rosErrorThrottle(
        kErrorPeriod, name_, "The timestamp of the motor command precedes the current time.");
      return;
    }

    const auto dt = (cur_time - last_cmd_time_).toSec();
    latency_filter_.update(latency->data, dt);
    const auto& filtered_latency = latency_filter_.getState();
    if (filtered_latency > kCheckLatencyThreshold)
    {
      rosWarnThrottle(
        kWarnPeriod, name_,
        "The time averaged latency from IMU to the motor command is "
          << filtered_latency << ", which exceeds the threshold " << kCheckLatencyThreshold);
    }
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

bool MotorsHandler::armRotorsCb(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res)
{
  if (!is_armed_ && req.data)
  {
    rosInfo(name_, "Arming rotors.");
    disarm_timer_.start();
    disarm_start_time_ = ros::Time::now();
  }
  else if (is_armed_ && !req.data)
  {
    rosInfo(name_, "Disarming rotors.");
    check_interval_timer_.stop();
    is_armed_ = false;
  }

  res.success = true;
  return true;
}

void MotorsHandler::setupPwmTimerCb(const ros::TimerEvent& event)
{
  // サービスサーバの起動を待つ
  if (!setup_pwm_sc_.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
  {
    rosWarn(
      name_, "Failed to connect to '" << tobas::kSetupPwmSrv << "' service server. Retrying...");
    return;
  }

  // PWMのセットアップ
  tobas_msgs::SetupPwm setup_pwm_msg;
  for (const auto& rotor_config : drone_.rotorConfigs())
  {
    const auto channel = channelFromPin(rotor_config.pin);
    setup_pwm_msg.request.channel = channel;
    setup_pwm_msg.request.frequency = kPwmFrequency;
    if (!setup_pwm_sc_.call(setup_pwm_msg) || !setup_pwm_msg.response.success)
    {
      rosWarn(name_, "Failed to setup RC output on CH" << channel << ". Retrying...");
      return;
    }
  }

  // PWMのセットアップ完了
  setup_pwm_timer_.stop();

  // Disarmを開始
  disarm_timer_.start();
  disarm_start_time_ = event.current_real;
}

void MotorsHandler::disarmTimerCb(const ros::TimerEvent& event)
{
  setPeriodOnAllChannels(kPwmDisarm);

  if ((event.current_real - disarm_start_time_).toSec() > kDisarmDuration)
  {
    is_armed_ = true;
    latency_filter_.initialize(kCheckLatencyTimeConst, 0.);

    // コマンドのインターバルチェックを開始
    disarm_timer_.stop();
    check_interval_timer_.start();

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

      cur_speeds_pub_.publish(real_speeds);
    }
  }
}
}  // namespace tobas_real

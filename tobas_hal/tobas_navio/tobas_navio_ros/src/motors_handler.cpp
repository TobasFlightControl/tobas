#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/algorithm.hpp>
#include <tobas_std_tools/vector.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_tools/utils.hpp>
#include <tobas_msgs/PwmArray.h>
#include <tobas_msgs/RotorSpeeds.h>
#include <tobas_msgs/EnablePwm.h>

#include "../include/tobas_navio_ros/motors_handler.hpp"
#include "../include/tobas_navio_ros/common.hpp"

using namespace std;
using namespace tobas_std;

namespace tobas_navio_ros
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

  get_arm_ss_ = nh_.advertiseService(tobas::kGetArmSrv, &self::getArmCb, this);
  set_arm_ss_ = nh_.advertiseService(tobas::kSetArmSrv, &self::setArmCb, this);
  enable_pwm_sc_ = nh_.serviceClient<tobas_msgs::EnablePwm>(tobas::kEnablePwmSrv);
  pre_arm_check_sc_ = nh_.serviceClient<std_srvs::Trigger>(tobas::kPreArmCheckSrv);

  check_interval_timer_ =
    nh_.createTimer(kCheckIntervalTimerRate, &self::checkIntervalTimerCb, this, false, false);

  publishArming();
}

void MotorsHandler::getRosParams()
{
}

void MotorsHandler::registerPublishers()
{
  pwms_pub_ = nh_.advertise<tobas_msgs::PwmArray>(tobas::kPwmCmdTopic, 1);
  arming_pub_ = nh_.advertise<std_msgs::Bool>(tobas::kArmingTopic, 1, true);
}

void MotorsHandler::registerSubscribers()
{
  tar_speeds_sub_ =
    nh_.subscribe(tobas::kRotorSpeedsCmdTopic, 1, &self::rotSpeedsCmdCb, this, tcpNoDelay());
  battery_sub_ = nh_.subscribe(tobas::kBatteryLpfTopic, 1, &self::batteryCb, this, tcpNoDelay());
}

bool MotorsHandler::armRotors()
{
  if (!enablePwms(true))
  {
    TOBAS_ERROR("Failed to enable PWMs.");
    return false;
  }

  const auto t_start = ros::Time::now();
  while ((ros::Time::now() - t_start).toSec() < tobas::kDisarmDuration)
  {
    setPeriodOnAllChannels(kPwmDisarm);
    ros::Duration(kDisarmInterval).sleep();
  }

  is_armed_ = true;
  check_interval_timer_.start();

  TOBAS_INFO("The motors are ready to rotate.");
  return true;
}

bool MotorsHandler::disarmRotors()
{
  if (!enablePwms(false))
  {
    TOBAS_ERROR("Failed to disable PWMs.");
    return false;
  }

  is_armed_ = false;
  check_interval_timer_.stop();

  return true;
}

bool MotorsHandler::enablePwms(const bool& enable)
{
  if (!enable_pwm_sc_.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
  {
    TOBAS_ERROR("Failed to connect to '", tobas::kEnablePwmSrv, "' server.");
    return false;
  }

  tobas_msgs::EnablePwm enable_pwm_msg;
  for (const auto& rotor : drone_.rotorConfigs())
  {
    enable_pwm_msg.request.channel = rotor.channel;
    enable_pwm_msg.request.enable = enable;
    if (!enable_pwm_sc_.call(enable_pwm_msg) || !enable_pwm_msg.response.success)
    {
      TOBAS_ERROR("Failed to enable/disable RC output CH", rotor.channel, ".");
      return false;
    }
  }

  return true;
}

bool MotorsHandler::preArmCheck()
{
  if (!pre_arm_check_sc_.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
  {
    TOBAS_ERROR("Failed to connect to '", tobas::kPreArmCheckSrv, "' server.");
    return false;
  }

  std_srvs::Trigger pre_arm_check_msg;
  if (!pre_arm_check_sc_.call(pre_arm_check_msg) || !pre_arm_check_msg.response.success)
  {
    error(pre_arm_check_msg.response.message);
    return false;
  }

  return true;
}

void MotorsHandler::setPeriodOnAllChannels(const double& period)
{
  const auto pwms = boost::make_shared<tobas_msgs::PwmArray>();
  pwms->header.stamp = ros::Time::now();
  for (const auto& rotor : drone_.rotorConfigs())
    pwms->pwm.emplace_back(rotor.channel, period);
  pwms_pub_.publish(pwms);
}

void MotorsHandler::publishArming()
{
  const auto arming_msg = boost::make_shared<std_msgs::Bool>();
  arming_msg->data = is_armed_;
  arming_pub_.publish(arming_msg);
}

void MotorsHandler::rotSpeedsCmdCb(const tobas_msgs::RotorSpeedsConstPtr& tar_speeds)
{
  if (!is_armed_)
    return;

  if (battery_ == nullptr)
  {
    TOBAS_ERROR_THROTTLE(
      kErrorPeriod, "The rotors cannot be rotated because battery state has not been received "
                    "yet.");
    return;
  }

  const auto data_size = tar_speeds->speeds.size();
  if (data_size != drone_.numRotors())
  {
    TOBAS_ERROR("Size mismatch: ", data_size, " != ", drone_.numRotors());
    return;
  }

  // Create PWM message
  const auto pwms = boost::make_shared<tobas_msgs::PwmArray>();
  pwms->header = tar_speeds->header;

  // Update PWM periods
  for (size_t rotor_idx = 0; rotor_idx < data_size; ++rotor_idx)
  {
    const auto& rotor = drone_.rotorConfig(rotor_idx);

    // 目標回転数を決定
    const auto max_speed = drone_.maxRotSpeed(rotor_idx, battery_->voltage);
    auto tar_speed = tar_speeds->speeds[rotor_idx];
    if (tar_speed < 0.)  // モータテストでも使用するため，ここではARM_THROTTLEの制約を課さない
    {
      warn(
        "Negative rotation speed is commanded on CH", rotor_idx, ": ", tar_speed, " < 0 [rad/s]");
      tar_speed = 0.;
    }
    else if (tar_speed > max_speed + tobas::kRotSpeedMargin)
    {
      warn(
        "Target rotation speed of CH", rotor_idx, " is too high: ", tar_speed, " > ", max_speed,
        " [rad/s]");
      tar_speed = max_speed;
    }

    // PWMコマンドメッセージを作成
    double pwm_period;
    switch (rotor.esc_signal_mode)
    {
      case tobas::EscSignalMode::BLHELI_OPEN_LOOP:
      {
        const auto throttle = drone_.throttleFromRotSpeed(rotor_idx, tar_speed, battery_->voltage);
        pwm_period = remap(throttle, tobas::kMinThrottle, tobas::kMaxThrottle, kPwmMin, kPwmMax);
        break;
      }
      case tobas::EscSignalMode::BLHELI_CLOSED_LOOP_LOW_RANGE:
      {
        const auto tar_erpm = drone_.erpmFromRotSpeed(rotor_idx, tar_speed);
        pwm_period = remap(tar_erpm, 0., kBLHeliClosedLoopLowRangeMaxERPM, kPwmMin, kPwmMax);
        break;
      }
      case tobas::EscSignalMode::BLHELI_CLOSED_LOOP_MID_RANGE:
      {
        const auto tar_erpm = drone_.erpmFromRotSpeed(rotor_idx, tar_speed);
        pwm_period = remap(tar_erpm, 0., kBLHeliClosedLoopMidRangeMaxERPM, kPwmMin, kPwmMax);
        break;
      }
      case tobas::EscSignalMode::BLHELI_CLOSED_LOOP_HIGH_RANGE:
      {
        const auto tar_erpm = drone_.erpmFromRotSpeed(rotor_idx, tar_speed);
        pwm_period = remap(tar_erpm, 0., kBLHeliClosedLoopHighRangeMaxERPM, kPwmMin, kPwmMax);
        break;
      }
      default:
      {
        TOBAS_ERROR("Unknown ESC signal mode of CH", rotor.channel);
        break;
      }
    }
    pwms->pwm.emplace_back(rotor.channel, pwm_period);
  }

  // Publish PWM commands
  pwms_pub_.publish(pwms);

  // Update last commanded time
  last_cmd_time_ = ros::Time::now();

  // Now the motors are activated
  is_activated_ = true;
}

void MotorsHandler::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  battery_ = battery;
}

bool MotorsHandler::getArmCb(tobas_msgs::GetArmRequest&, tobas_msgs::GetArmResponse& res)
{
  res.arming = is_armed_;
  return true;
}

bool MotorsHandler::setArmCb(tobas_msgs::SetArmRequest& req, tobas_msgs::SetArmResponse& res)
{
  res.success = false;

  if (!is_armed_ && req.arming)
  {
    if (!req.ignore_pre_arm_check && !preArmCheck())
    {
      res.message = "Pre-arm check failed.";
      return true;
    }

    if (!armRotors())
    {
      res.message = "Failed to enable PWMs.";
      return true;
    }
  }
  else if (is_armed_ && !req.arming)
  {
    if (!disarmRotors())
    {
      res.message = "Failed to disable PWMs.";
      return true;
    }
  }

  publishArming();

  res.success = true;
  return true;
}

void MotorsHandler::checkIntervalTimerCb(const ros::TimerEvent& event)
{
  const auto time_after_last_cmd = (event.current_real - last_cmd_time_).toSec();
  if (time_after_last_cmd > tobas::kAutoResetTimeThreshold)
  {
    setPeriodOnAllChannels(kPwmMin);
    if (is_activated_)
    {
      is_activated_ = false;
      warn(
        "The speeds of all rotors are automatically stopped because ",
        tobas::kAutoResetTimeThreshold, " seconds have elapsed since the last command.");
    }
  }
}
}  // namespace tobas_navio_ros

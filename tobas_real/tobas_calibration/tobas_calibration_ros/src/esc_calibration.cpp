#include <tobas_std_tools/time.hpp>
#include <tobas_ros_tools/exception.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_navio_ros/common.hpp>
#include <tobas_msgs/PwmArray.h>
#include <tobas_msgs/GetArm.h>
#include <tobas_msgs/EnablePwm.h>

#include "../include/tobas_calibration_ros/esc_calibration.hpp"

using namespace std;

namespace tobas_calibration
{
EscCalibrationRos::EscCalibrationRos(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name),
    as_(nh, kActionName, boost::bind(&EscCalibrationRos::executeCb, this, _1), false)
{
  drone_.loadFromParam(nh_);

  if (adc_.initialize() < 0)
    TOBAS_EXIT("Failed to initialize ADC driver.");

  pwms_pub_ = nh_.advertise<tobas_msgs::PwmArray>(tobas::kPwmCmdTopic, 1);
  get_arm_sc_ = nh_.serviceClient<tobas_msgs::GetArm>(tobas::kGetArmSrv);
  enable_pwm_sc_ = nh_.serviceClient<tobas_msgs::EnablePwm>(tobas::kEnablePwmSrv);

  as_.start();
}

void EscCalibrationRos::sendMaximum()
{
  const auto start_time = ros::Time::now();
  while ((ros::Time::now() - start_time).toSec() < kSleepHigh)
    setPeriodAndSleep(tobas_navio_ros::kPwmMax);
}

void EscCalibrationRos::sendMinimum()
{
  const auto start_time = ros::Time::now();
  while ((ros::Time::now() - start_time).toSec() < kSleepLow)
    setPeriodAndSleep(tobas_navio_ros::kPwmMin);
}

void EscCalibrationRos::setPeriod(const double& period)
{
  const auto pwms = boost::make_shared<tobas_msgs::PwmArray>();
  pwms->header.stamp = ros::Time::now();
  for (const auto& rotor : drone_.rotorConfigs())
    pwms->pwm.emplace_back(rotor.channel, period);
  pwms_pub_.publish(pwms);
}

void EscCalibrationRos::setPeriodAndSleep(const double& period)
{
  setPeriod(period);
  tobas_std::msleep(kInterval);
}

bool EscCalibrationRos::isBatteryConnected()
{
  const auto a2_value = adc_.read(tobas_navio_ros::kPowerModuleVoltageChannel);
  return a2_value > kA2ValueThreshold;
}

bool EscCalibrationRos::checkDisarmed()
{
  tobas_msgs::GetArm get_arm_msg;
  if (!get_arm_sc_.call(get_arm_msg))
  {
    as_.setAborted(result_, "Failed to get arming state.");
    return false;
  }
  if (get_arm_msg.response.arming)
  {
    as_.setAborted(result_, "Cannot execute ESC calibration because the motors are armed now.");
    return false;
  }

  return true;
}

bool EscCalibrationRos::enablePWM()
{
  tobas_msgs::EnablePwm enable_pwm_msg;
  enable_pwm_msg.request.enable = true;

  for (const auto& rotor : drone_.rotorConfigs())
  {
    enable_pwm_msg.request.channel = rotor.channel;
    if (!enable_pwm_sc_.call(enable_pwm_msg) || !enable_pwm_msg.response.success)
    {
      as_.setAborted(result_, "Failed to enable PWM of CH" + to_string(rotor.channel) + ".");
      return false;
    }
  }

  return true;
}

void EscCalibrationRos::disablePWM()
{
  tobas_msgs::EnablePwm enable_pwm_msg;
  enable_pwm_msg.request.enable = false;

  for (const auto& rotor : drone_.rotorConfigs())
  {
    enable_pwm_msg.request.channel = rotor.channel;
    if (!enable_pwm_sc_.call(enable_pwm_msg) || !enable_pwm_msg.response.success)
      TOBAS_ERROR("Failed to disable PWM of CH", rotor.channel, ".");
  }
}

void EscCalibrationRos::executeCb(const GoalType::ConstPtr&)
{
  const auto action_called_time = ros::Time::now();

  // 各サービスサーバへの接続をチェック
  if (!get_arm_sc_.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
  {
    as_.setAborted(
      result_, "Failed to connect to " + string(tobas::kGetArmSrv) + " service server.");
    return;
  }
  if (!enable_pwm_sc_.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
  {
    as_.setAborted(
      result_, "Failed to connect to " + string(tobas::kEnablePwmSrv) + " service server.");
    return;
  }

  // アームされていないことを確認
  if (!checkDisarmed())
    return;

  // バッテリーが接続されていないことを確認
  if (isBatteryConnected())
  {
    as_.setAborted(result_, "Please disconnect battery before starting ESC calibration.");
    return;
  }

  // PWMを有効化
  if (!enablePWM())
    return;

  // バッテリーが接続されるのを待つ
  TOBAS_INFO("Waiting for battery connection.");
  while (!isBatteryConnected())
  {
    if ((ros::Time::now() - action_called_time).toSec() > kTimeout)
    {
      disablePWM();
      as_.setAborted(result_, "Battery connection is not detected before timeout.");
      return;
    }
    setPeriodAndSleep(tobas_navio_ros::kPwmMax);
  }

  // 最大スロットルを指令
  TOBAS_INFO("Sending maximum throttle.");
  sendMaximum();

  // 最小スロットルを指令
  TOBAS_INFO("Sending minimum throttle.");
  sendMinimum();

  // PWMを無効化
  disablePWM();

  as_.setSucceeded(result_);
}
}  // namespace tobas_calibration

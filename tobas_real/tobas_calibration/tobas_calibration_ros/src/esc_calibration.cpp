#include <tobas_std_tools/time.hpp>
#include <tobas_ros_tools/util.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_real_ros/common.hpp>
#include <tobas_msgs/PwmArray.h>
#include <tobas_msgs/GetArm.h>
#include <tobas_msgs/EnablePwm.h>

#include "../include/tobas_calibration_ros/esc_calibration.hpp"

using namespace std;

namespace tobas_calibration
{
EscCalibrationRos::EscCalibrationRos(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name), as_(nh, kActionName, boost::bind(&EscCalibrationRos::executeCb, this, _1), false)
{
  drone_.loadFromParam(nh_);

  pwms_pub_ = nh_.advertise<tobas_msgs::PwmArray>(tobas::kPwmCmdTopic, 1);
  get_arm_sc_ = nh_.serviceClient<tobas_msgs::GetArm>(tobas::kGetArmSrv);
  enable_pwm_sc_ = nh_.serviceClient<tobas_msgs::EnablePwm>(tobas::kEnablePwmSrv);

  as_.start();
}

void EscCalibrationRos::sendMaximum()
{
  const auto start_time = ros::Time::now();
  while ((ros::Time::now() - start_time).toSec() < kHighDuration)
    setPeriodAndSleep(tobas::kPwmMax);
}

void EscCalibrationRos::sendMinimum()
{
  const auto start_time = ros::Time::now();
  while ((ros::Time::now() - start_time).toSec() < kLowDuration)
    setPeriodAndSleep(tobas::kPwmMin);
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

bool EscCalibrationRos::checkBatteryDisconnected()
{
  tobas_msgs::Battery battery;

  // バッテリー状態を取得
  if (!tobas_ros::subscribeOnce(battery, tobas::kBatteryTopic, nh_, kTimeout))
  {
    as_.setAborted(result_, "Failed to receive battery status.");
    return false;
  }

  // バッテリー電圧が閾値以下であることを確認
  if (battery.voltage > kVoltageThreshold)
  {
    as_.setAborted(result_, "Please disconnect battery before starting ESC calibration.");
    return false;
  }

  return true;
}

bool EscCalibrationRos::waitForBatteryConnection()
{
  // バッテリーメッセージを初期化
  battery_ = nullptr;

  // 一時的にバッテリーの購読を開始
  const auto battery_sub = nh_.subscribe(tobas::kBatteryTopic, 1, &EscCalibrationRos::batteryCb, this);

  // バッテリー電圧が閾値を超えるまで最大値を指令し続ける
  const auto start_time = ros::Time::now();
  while (battery_ != nullptr && battery_->voltage < kVoltageThreshold)
  {
    if ((ros::Time::now() - start_time).toSec() > kTimeout)
    {
      disablePWM();
      as_.setAborted(result_, "Battery connection is not detected before timeout.");
      return false;
    }
    setPeriodAndSleep(tobas::kPwmMax);
    ros::spinOnce();
  }

  return true;
}

void EscCalibrationRos::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  battery_ = battery;
}

void EscCalibrationRos::executeCb(const GoalType::ConstPtr&)
{
  // 各サービスサーバへの接続をチェック
  if (!get_arm_sc_.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
  {
    as_.setAborted(result_, "Failed to connect to " + string(tobas::kGetArmSrv) + " service server.");
    return;
  }
  if (!enable_pwm_sc_.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
  {
    as_.setAborted(result_, "Failed to connect to " + string(tobas::kEnablePwmSrv) + " service server.");
    return;
  }

  // アームされていないことを確認
  if (!checkDisarmed())
    return;

  // バッテリーが接続されていないことを確認
  if (!checkBatteryDisconnected())
    return;

  // PWMを有効化
  if (!enablePWM())
    return;

  // バッテリーが接続されるのを待つ
  TOBAS_INFO("Waiting for battery connection.");
  if (!waitForBatteryConnection())
    return;

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

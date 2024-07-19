#include <tobas_std_tools/time.hpp>
#include <tobas_ros_tools/util.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_real_ros/common.hpp>
#include <tobas_msgs/ThrottleArray.h>
#include <tobas_msgs/GetArm.h>
#include <tobas_msgs/EnableRCOutput.h>

#include "../include/tobas_calibration_ros/esc_calibration.hpp"

using namespace std;

namespace tobas_calibration
{
EscCalibrationRos::EscCalibrationRos(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name), as_(nh, kActionName, boost::bind(&EscCalibrationRos::executeCb, this, _1), false)
{
  drone_.loadFromParam(nh_);

  throttles_pub_ = nh_.advertise<tobas_msgs::ThrottleArray>(tobas::kThrottlesCmdTopic, 1);
  get_arm_sc_ = nh_.serviceClient<tobas_msgs::GetArm>(tobas::kGetArmSrv);
  enable_rcout_sc_ = nh_.serviceClient<tobas_msgs::EnableRCOutput>(tobas::kEnableRcOutputSrv);

  as_.start();
}

void EscCalibrationRos::sendMaximum()
{
  const auto start_time = ros::Time::now();
  while ((ros::Time::now() - start_time).toSec() < kHighDuration)
    setThrottleAndSleep(tobas::kMaxThrottle);
}

void EscCalibrationRos::sendMinimum()
{
  const auto start_time = ros::Time::now();
  while ((ros::Time::now() - start_time).toSec() < kLowDuration)
    setThrottleAndSleep(tobas::kMinThrottle);
}

void EscCalibrationRos::setThrottle(const double& throttle)
{
  const auto throttles = boost::make_shared<tobas_msgs::ThrottleArray>();
  throttles->header.stamp = ros::Time::now();
  for (const auto& rotor : drone_.rotorConfigs())
    throttles->throttles.emplace_back(rotor.channel, throttle);
  throttles_pub_.publish(throttles);
}

void EscCalibrationRos::setThrottleAndSleep(const double& throttle)
{
  setThrottle(throttle);
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

bool EscCalibrationRos::enableRCOutput(bool enable)
{
  tobas_msgs::EnableRCOutput enable_rcout_msg;
  enable_rcout_msg.request.enable = enable;

  for (const auto& rotor : drone_.rotorConfigs())
  {
    enable_rcout_msg.request.channel = rotor.channel;

    if (!enable_rcout_sc_.call(enable_rcout_msg))
    {
      as_.setAborted(result_, "Failed to call EnableRCOutput service.");
      return false;
    }

    if (!enable_rcout_msg.response.success)
    {
      as_.setAborted(result_, enable_rcout_msg.response.message);
      return false;
    }
  }

  return true;
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
  while (battery_ == nullptr || battery_->voltage < kVoltageThreshold)
  {
    if ((ros::Time::now() - start_time).toSec() > kTimeout)
    {
      enableRCOutput(false);
      as_.setAborted(result_, "Battery connection is not detected before timeout.");
      return false;
    }
    setThrottleAndSleep(tobas::kMaxThrottle);
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
  if (!enable_rcout_sc_.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
  {
    as_.setAborted(result_, "Failed to connect to " + string(tobas::kEnableRcOutputSrv) + " service server.");
    return;
  }

  // アームされていないことを確認
  if (!checkDisarmed())
    return;

  // バッテリーが接続されていないことを確認
  if (!checkBatteryDisconnected())
    return;

  // RC出力を有効化
  if (!enableRCOutput(true))
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

  // RC出力を無効化
  enableRCOutput(false);

  as_.setSucceeded(result_);
}
}  // namespace tobas_calibration

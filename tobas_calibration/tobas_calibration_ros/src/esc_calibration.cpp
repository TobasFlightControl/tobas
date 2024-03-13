#include <tobas_ros_tools/exception.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_real/common.hpp>
#include <tobas_msgs/PwmArray.h>

#include "../include/tobas_calibration/esc_calibration.hpp"

using namespace std;

namespace tobas_calibration
{
EscCalibrationRos::EscCalibrationRos(ros::NodeHandle& nh)
  : as_(nh, kActionName, boost::bind(&EscCalibrationRos::executeCb, this, _1), false)
{
  drone_.loadFromParam(nh);

  if (adc_.initialize() < 0)
    ROS_EXIT(nh, "Failed to initialize ADC driver.");

  pwms_pub_ = nh.advertise<tobas_msgs::PwmArray>(tobas::kPwmCmdTopic, 1);
  as_.start();
}

void EscCalibrationRos::sendMaximum()
{
  const auto start_time = ros::Time::now();
  while ((ros::Time::now() - start_time).toSec() < kSleepHigh)
    setPeriodAndSleep(tobas_real::kPwmMax);
}

void EscCalibrationRos::sendMinimum()
{
  const auto start_time = ros::Time::now();
  while ((ros::Time::now() - start_time).toSec() < kSleepLow)
    setPeriodAndSleep(tobas_real::kPwmMin);
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
  usleep(kInterval);
}

bool EscCalibrationRos::isBatteryConnected()
{
  const auto a2_value = adc_.read(tobas_real::kPowerModuleVoltageChannel);
  return a2_value > kA2ValueThreshold;
}

void EscCalibrationRos::executeCb(const GoalType::ConstPtr&)
{
  if (isBatteryConnected())
  {
    as_.setAborted(result_, "Please disconnect battery before starting ESC calibration.");
    return;
  }

  // バッテリーが接続されるのを待つ
  ROS_INFO("Waiting for battery connection.");
  const auto action_called_time = ros::Time::now();
  while (!isBatteryConnected())
  {
    if ((ros::Time::now() - action_called_time).toSec() < kTimeout)
    {
      as_.setAborted(result_, "Battery connection is not detected before timeout.");
      return;
    }
    setPeriodAndSleep(tobas_real::kPwmMax);
  }

  // 最大スロットルを指令
  ROS_INFO("Sending maximum throttle.");
  sendMaximum();

  // 最小スロットルを指令
  ROS_INFO("Sending minimum throttle.");
  sendMinimum();

  as_.setSucceeded(result_);
}
}  // namespace tobas_calibration

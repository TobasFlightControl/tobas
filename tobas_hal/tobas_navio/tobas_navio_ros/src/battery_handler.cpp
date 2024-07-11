#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/Adc.h>

#include "../include/tobas_navio_ros/battery_handler.hpp"
#include "../include/tobas_navio_ros/common.hpp"

using namespace std;

namespace tobas_navio_ros
{
BatteryHandler::BatteryHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name) : super(nh, pnh, name)
{
  if (!adc_.initialize())
    TOBAS_EXIT("Failed to initialize ADC driver.");

  adc_pub_ = nh_.advertise<tobas_hal_msgs::Adc>(hal::kAdcTopic, 1);
  main_timer_ = nh_.createTimer(kSamplingRate, &self::mainTimerCb, this);
}

bool BatteryHandler::getVoltage(double& voltage)
{
  // Read from ADC converter
  const auto a2_value = adc_.read(kPowerModuleVoltageChannel);
  if (a2_value < 0)
  {
    TOBAS_ERROR("Failed to read battery voltage.");
    return false;
  }

  // Compute voltage
  voltage = static_cast<double>(a2_value) * 1e-3;

  return true;
}

bool BatteryHandler::getCurrent(double& current)
{
  // Read from ADC converter
  const auto a3_value = adc_.read(kPowerModuleCurrentChannel);
  if (a3_value < 0)
  {
    TOBAS_ERROR("Failed to read battery current.");
    return false;
  }

  // Compute current
  current = static_cast<double>(a3_value) * 1e-3;

  return true;
}

void BatteryHandler::mainTimerCb(const ros::TimerEvent& event)
{
  // Create battery message
  const auto adc_msg = boost::make_shared<tobas_hal_msgs::Adc>();
  adc_msg->header.stamp = event.current_real;

  // Fill values
  if (!getVoltage(adc_msg->voltage))
    return;
  if (!getCurrent(adc_msg->current))
    return;

  // Publish battery message
  adc_pub_.publish(adc_msg);
}
}  // namespace tobas_navio_ros

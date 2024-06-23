#include <tobas_tools/constants.hpp>
#include <tobas_msgs/Battery.h>

#include "../include/tobas_navio_ros/battery_handler.hpp"
#include "../include/tobas_navio_ros/common.hpp"

using namespace std;

namespace tobas_navio_ros
{
BatteryHandler::BatteryHandler(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name), property_client_(nh_, kPropertyNamespace)
{
  PRINT_DEBUG("BatteryHandler::BatteryHandler");

  if (adc_.initialize() < 0)
    TOBAS_EXIT("Failed to initialize ADC driver.");

  reloadConfig();

  battery_pub_ = nh_.advertise<tobas_msgs::Battery>(tobas::kBatteryTopic, 1);
  reload_config_srv_ = nh_.advertiseService(name + tobas::kReloadConfigSrvSuffix, &self::reloadConfigCb, this);
  main_timer_ = nh_.createTimer(kSamplingRate, &self::mainTimerCb, this);

  PRINT_DEBUG("/BatteryHandler::BatteryHandler");
}

bool BatteryHandler::reloadConfig()
{
  if (property_client_.get(kConfigKey_AdcCoef, adc_coef_) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    adc_coef_ = kDefaultAdcVoltageCoef;
    return false;
  }

  return true;
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
  voltage = static_cast<double>(a2_value) * adc_coef_ * 1e-3;

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
  current = static_cast<double>(a3_value) * kAdcCurrentCoef * 1e-3;

  return true;
}

bool BatteryHandler::reloadConfigCb(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& res)
{
  if (!reloadConfig())
  {
    res.success = false;
    res.message = "Failed to reload configurations.";
    return true;
  }

  res.success = true;
  return true;
}

void BatteryHandler::mainTimerCb(const ros::TimerEvent& event)
{
  // Create battery message
  const auto battery_msg = boost::make_shared<tobas_msgs::Battery>();
  battery_msg->header.stamp = event.current_real;

  // Fill values
  if (!getVoltage(battery_msg->voltage))
    return;
  if (!getCurrent(battery_msg->current))
    return;

  // Publish battery message
  battery_pub_.publish(battery_msg);
}
}  // namespace tobas_navio_ros

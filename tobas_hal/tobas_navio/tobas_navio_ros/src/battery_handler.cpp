#include <tobas_std_tools/property_tree.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/Battery.h>

#include "../include/tobas_navio_ros/battery_handler.hpp"
#include "../include/tobas_navio_ros/common.hpp"

using namespace std;

namespace tobas_navio_ros
{
BatteryHandler::BatteryHandler(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name)
{
  getRosParams();

  if (!reloadConfig())
    exit("Failed to load configuratins.");

  if (adc_.initialize() < 0)
    exit("Failed to initialize ADC driver.");

  registerPublishers();
  registerSubscribers();

  reload_config_srv_ =
    nh_.advertiseService(name + tobas::kReloadConfigSrvSuffix, &self::reloadConfigCb, this);
  main_timer_ = nh_.createTimer(kSamplingRate, &self::mainTimerCb, this);
}

void BatteryHandler::getRosParams()
{
}

void BatteryHandler::registerPublishers()
{
  battery_pub_ = nh_.advertise<tobas_msgs::Battery>(tobas::kBatteryTopic, 1);
}

void BatteryHandler::registerSubscribers()
{
}

bool BatteryHandler::reloadConfig()
{
  tobas_std::PropertyTree pt(kConfigPath);
  pt.get(kConfigKey_AdcCoef, adc_coef_, kDefaultAdcVoltageCoef);

  return true;
}

bool BatteryHandler::getVoltage(double& voltage)
{
  // Read from ADC converter
  const auto a2_value = adc_.read(kPowerModuleVoltageChannel);
  if (a2_value < 0)
  {
    error("Failed to read battery voltage.");
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
    error("Failed to read battery current.");
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

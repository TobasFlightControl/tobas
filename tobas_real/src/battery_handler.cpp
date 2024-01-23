#include <tobas_std_tools/property_tree.hpp>
#include <tobas_ros_tools/console_message.hpp>
#include <tobas_ros_tools/exception.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_msgs/Battery.h>

#include "../include/tobas_real/battery_handler.hpp"
#include "../include/tobas_real/common.hpp"

using namespace std;

namespace tobas_real
{
BatteryHandler::BatteryHandler(
  const ros::NodeHandle& nh,
  const ros::NodeHandle& pnh,
  const string& name)
  : super(nh, pnh, name)
{
  getRosParams();
  getAdcCoefficient();

  adc_.initialize();

  registerPublishers();
  registerSubscribers();

  main_timer_ = nh_.createTimer(kUpdateRate, &self::mainTimerCb, this);
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
  super::registerSubscribers();
}

void BatteryHandler::getAdcCoefficient()
{
  tobas_std::PropertyTree pt(kConfigPath);
  pt.get(kConfigKey_AdcCoef, adc_coef_);
}

void BatteryHandler::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::STOP:
      nh_.shutdown();
      main_timer_.stop();
      break;
    default:
      break;
  }
}

void BatteryHandler::mainTimerCb(const ros::TimerEvent& event)
{
  // Read from ADC converter
  const int a2_value = adc_.read(kPowerModuleVoltageChannel);
  if (a2_value < 0)
  {
    rosError(name_, "Failed to read battery voltage.");
    return;
  }

  // Compute voltage
  const double voltage = static_cast<double>(a2_value) * adc_coef_ * 1e-3;
  if (voltage < kVoltageThreshold)
  {
    rosErrorThrottle(
      kErrorPeriod, name_,
      "Battery voltage is abnormal: " << voltage << "V. Please check the ADC connection.");
    return;
  }

  // Create battery message
  const auto battery_msg = boost::make_shared<tobas_msgs::Battery>();
  battery_msg->header.stamp = event.current_real;
  battery_msg->voltage = voltage;
  battery_msg->current = nan(tobas::kUnknown);

  // Publish battery message
  battery_pub_.publish(battery_msg);
}
}  // namespace tobas_real

#include <boost/property_tree/ini_parser.hpp>

#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include <tobas_msgs/Battery.h>

#include "../include/tobas_real/battery_handler.hpp"
#include "../include/tobas_real/common.hpp"

using namespace std;

namespace tobas_real
{
BatteryHandler::BatteryHandler(const ros::NodeHandle& nh, const ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name)
{
  getRosParams();
  getAdcCoefficient();

  adc_.initialize();

  registerPublishers();
  registerSubscribers();

  main_timer_ = nh_.createTimer(kUpdateRate, &BatteryHandler::mainTimerCb, this);
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
  event_sub_ = nh_.subscribe(tobas::kEventTopic, 1, &BatteryHandler::eventCb, this, tcpNoDelay());
}

void BatteryHandler::getAdcCoefficient()
{
  boost::property_tree::ptree pt;
  boost::property_tree::ini_parser::read_ini(kConfigPath, pt);

  adc_coef_ = pt.get<double>(kConfigKey_AdcCoef);
  if (adc_coef_ <= 0.)
  {
    ROS_THROW_NAMED(name_, "Negative ADC coefficient: " << adc_coef_);
  }

  rosInfo(name_, "ADC coefficient: " << adc_coef_);
}

void BatteryHandler::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      nh_.shutdown();
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
  const double voltage_raw = static_cast<double>(a2_value) * adc_coef_ * 1e-3;
  if (voltage_raw < kVoltageThreshold)
  {
    rosErrorThrottle(
      kErrorPeriod, name_,
      "Battery voltage is abnormal: " << voltage_raw << "V. Please check the ADC connection.");
    return;
  }

  // Filtering
  if (lpf_.isInitialized())
  {
    const auto ts = (event.current_real - event.last_real).toSec();
    lpf_.update(voltage_raw, ts);
  }
  else
  {
    lpf_.initialize(kLpfTimeConst, voltage_raw);
  }

  // Create battery message
  const auto battery_msg = boost::make_shared<tobas_msgs::Battery>();
  battery_msg->header.stamp = event.current_real;
  battery_msg->voltage = lpf_.getState();

  // Publish battery message
  battery_pub_.publish(battery_msg);
}
}  // namespace tobas_real

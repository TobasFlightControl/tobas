#include <boost/property_tree/ini_parser.hpp>

#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>
#include <dh_ros_tools/rate.hpp>

#include <tobas_msgs/Battery.h>

#include "../include/tobas_real/battery_handler.hpp"
#include "../include/tobas_real/common.hpp"

using namespace std;

namespace tobas_real
{
BatteryHandler::BatteryHandler(ros::NodeHandle nh, ros::NodeHandle pnh) : super(nh, pnh)
{
  getRosParams();

  getAdcCoefficient();
  adc_.initialize();

  registerPublishers();
  registerSubscribers();

  main_timer_ = nh_.createTimer(ros::Duration(1 / kUpdateRate), &BatteryHandler::mainTimerCb, this);
}

void BatteryHandler::getRosParams()
{
}

void BatteryHandler::registerPublishers()
{
  battery_pub_ = nh_.advertise<tobas_msgs::Battery>("battery", 1);
}

void BatteryHandler::registerSubscribers()
{
  event_sub_ = nh_.subscribe("event", 1, &BatteryHandler::eventCb, this);
}

void BatteryHandler::getAdcCoefficient()
{
  boost::property_tree::ptree pt;
  boost::property_tree::ini_parser::read_ini(kConfigPath, pt);

  adc_coef_ = pt.get<double>(kConfigKey_AdcCoef);
  if (adc_coef_ <= 0.)
  {
    rosthrow("Negative ADC coefficient: " << adc_coef_);
  }

  rosInfo("ADC coefficient: " << adc_coef_);
}

void BatteryHandler::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      // nh_.shutdown();
      break;
    default:
      break;
  }
}

void BatteryHandler::mainTimerCb(const ros::TimerEvent& event)
{
  // Read battery voltage
  const int a2_value = adc_.read(kPowerModuleVoltageChannel);
  if (a2_value < 0)
  {
    rosError("Failed to read battery voltage.");
    return;
  }

  // Creata battery message
  const auto battery_msg = boost::make_shared<tobas_msgs::Battery>();
  battery_msg->header.stamp = event.current_real;
  battery_msg->voltage = static_cast<double>(a2_value) * adc_coef_ * 1e-3;

  // Publish battery message
  battery_pub_.publish(battery_msg);
}
}  // namespace tobas_real

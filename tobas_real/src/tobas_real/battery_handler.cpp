#include <boost/property_tree/ini_parser.hpp>

#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>
#include <dh_ros_tools/rate.hpp>

#include "../../include/tobas_real/battery_handler.hpp"
#include "../../include/tobas_real/common.hpp"

using namespace std;

namespace tobas_real
{
BatteryHandler::BatteryHandler()
{
  getRosParams();

  getAdcCoefficient();
  adc_.initialize();

  registerPublishers();
  registerSubscribers();
}

void BatteryHandler::run()
{
  dh_ros::Rate rate(kUpdateRate);

  while (ros::ok())
  {
    // Read battery voltage
    const int a2_value = adc_.read(kPowerModuleVoltageChannel);
    if (a2_value < 0)
    {
      rosError("Failed to read battery voltage.");
      continue;
    }

    // Fill battery message
    battery_msg_.header.stamp = ros::Time::now();
    battery_msg_.voltage = static_cast<double>(a2_value) * adc_coef_ * 1e-3;

    // Publish battery message
    battery_pub_.publish(battery_msg_);

    ros::spinOnce();
    rate.sleep();
  }
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

  adc_coef_ = pt.get<double>(kConfigKey_AdcCoef, kDefaultAdcCoef);
  if (adc_coef_ <= 0.)
  {
    rosthrow("Negative ADC coefficient: " << adc_coef_);
  }

  rosInfo("ADC coefficient: " << adc_coef_);
}

void BatteryHandler::eventCb(const tobas_msgs::Event& event)
{
  switch (event.data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      // ros::shutdown();
      break;
    default:
      break;
  }
}
}  // namespace tobas_real

#include <dh_ros_tools/rosparam.hpp>
#include <dh_ros_tools/rate.hpp>
#include <dh_ros_tools/exception.hpp>

#include "../../include/tobas_real/battery_handler.hpp"

#define POWER_MODULE_VOLTAGE_CHANNEL 2
#define VOLTAGE_MULTIPLIER 11.3
#define FREQ 100.  // [Hz]

namespace tobas_real
{
BatteryHandler::BatteryHandler()
{
  getRosParams();
  adc_.initialize();
  registerPublishers();
}

void BatteryHandler::run()
{
  dh_ros::Rate rate(FREQ);

  while (ros::ok())
  {
    // Access battery voltage channel
    const int a2_value = adc_.read(POWER_MODULE_VOLTAGE_CHANNEL);
    if (a2_value < 0)
    {
      rosError("Failed to read battery voltage.");
      continue;
    }

    // Fill battery message
    battery_msg_.header.stamp = ros::Time::now();
    battery_msg_.voltage = static_cast<double>(a2_value) * VOLTAGE_MULTIPLIER;

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
}
}  // namespace tobas_real

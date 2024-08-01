#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/Adc.h>

#include "../include/tobas_a1_ros/adc_driver.hpp"

using namespace std;

namespace a1
{
ADCDriver::ADCDriver(, const string& name) : super(node, pnh, name)
{
  if (!adc_.initialize())
    TOBAS_EXIT("Failed to initialize ADC.");

  adc_pub_ = node_.advertise<tobas_hal_msgs::Adc>(hal::kAdcTopic, 1);
  main_timer_ = node_.createTimer(kSamplingRate, &self::mainTimerCb, this);
}

void ADCDriver::mainTimerCb(const rclcpp::TimerEvent& event)
{
  // Create messages
  const auto msg = boost::make_shared<tobas_hal_msgs::Adc>();

  // Fill headers
  msg->header.stamp = event.current_real;

  // Read ADC
  if (!adc_.readVoltage(msg->voltage))
  {
    TOBAS_FATAL("Failed to read voltage.");
    return;
  }
  msg->current = 0.;  // TODO

  // Publish message
  adc_pub_.publish(msg);
}
}  // namespace a1

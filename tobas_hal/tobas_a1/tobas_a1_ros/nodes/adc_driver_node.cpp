#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/Adc.h>

#include <tobas_hal_core/base_sensor_node.hpp>
#include <tobas_a1_core/ads1220.hpp>

#include "../include/tobas_a1_ros/adc_driver.hpp"

using namespace std;

class ADCDriver : public hal::BaseSensorNode
{
  static constexpr size_t kSamplingRate = 100;  // [Hz]

  using self = ADCDriver;
  using super = hal::BaseSensorNode;

public:
  explicit ADCDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ADS1220 adc_;
  PublisherPtr<> adc_pub_;

  void mainTimerCb(const rclcpp::TimerEvent& event);
};

ADCDriver::ADCDriver(const rclcpp::NodeOptions& options) : super(name, options)
{
  if (!adc_.initialize())
    TOBAS_EXIT("Failed to initialize ADC.");

  adc_pub_ = createPublisher<tobas_hal_msgs::Adc>(hal::kAdcTopic);
  main_timer_ = node_.createTimer(kSamplingRate, &self::mainTimerCb, this);
}

void ADCDriver::mainTimerCb(const rclcpp::TimerEvent& event)
{
  // Create messages
  const auto msg =std::make_unique<tobas_hal_msgs::Adc>();

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
  adc_pub_->publish(msg);
}

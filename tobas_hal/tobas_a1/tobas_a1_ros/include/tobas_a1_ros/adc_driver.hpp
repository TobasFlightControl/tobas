#pragma once

#include <tobas_hal_core/base_sensor_node.hpp>
#include <tobas_a1_core/ads1220.hpp>

namespace a1
{
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
}  // namespace a1

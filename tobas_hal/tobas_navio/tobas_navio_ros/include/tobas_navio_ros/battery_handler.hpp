#pragma once

#include <tobas_hal_core/base_sensor_node.hpp>
#include <tobas_navio_core/adc.hpp>

namespace tobas_navio_ros
{
class BatteryHandler : public hal::BaseSensorNode
{
  static constexpr size_t kSamplingRate = 100;  // [Hz]

  // https://docs.emlid.com/tobas_navio_core/dev/adc/
  static constexpr size_t kPowerModuleVoltageChannel = 2;
  static constexpr size_t kPowerModuleCurrentChannel = 3;

  using self = BatteryHandler;
  using super = hal::BaseSensorNode;

public:
  explicit BatteryHandler(
    ros::NodeHandle& nh,
    ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  navio::ADC adc_;
  ros::Publisher adc_pub_;

  bool getVoltage(double& voltage);
  bool getCurrent(double& current);

  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_navio_ros

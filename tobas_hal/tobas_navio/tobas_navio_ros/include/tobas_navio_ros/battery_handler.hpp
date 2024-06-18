#pragma once

#include <std_srvs/Trigger.h>

#include <tobas_navio_core/adc.hpp>

#include "./base_sensor_node.hpp"

namespace tobas_navio_ros
{
class BatteryHandler : public BaseSensorNode
{
  // Constants
  static constexpr size_t kSamplingRate = 100;    // [Hz]
  static constexpr double kAdcCurrentCoef = 17.;  // https://docs.emlid.com/navio2/dev/adc/

  // Defaults
  static constexpr double kDefaultAdcVoltageCoef = 11.3;

  using self = BatteryHandler;
  using super = BaseSensorNode;

public:
  explicit BatteryHandler(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  navio::ADC adc_;

  // Config
  double adc_coef_;

  ros::Publisher battery_pub_;
  ros::ServiceServer reload_config_srv_;

  bool reloadConfig();
  bool getVoltage(double& voltage);
  bool getCurrent(double& current);

  bool reloadConfigCb(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_navio_ros

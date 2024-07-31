#pragma once

#include <std_srvs/Trigger.h>

#include <tobas_property_tools/property_client.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_hal_msgs/Adc.h>

namespace tobas_real_ros
{
class BatteryHandler : public tobas::BaseNode
{
  // Defaults (cf. ADC example: https://docs.emlid.com/navio2/dev/adc)
  static constexpr double kDefaultAdcVoltageCoef = 11.3;
  static constexpr double kDefaultAdcCurrentCoef = 17.0;

  using self = BatteryHandler;
  using super = tobas::BaseNode;

public:
  explicit BatteryHandler(
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

private:
  // Config
  double voltage_coef_ = kDefaultAdcVoltageCoef;
  double current_coef_ = kDefaultAdcCurrentCoef;

  ptree::PropertyClient property_client_;

  rclcpp::Publisher battery_pub_;
  rclcpp::Subscriber adc_sub_;

  rclcpp::ServiceServer reload_config_srv_;

  bool reloadConfig();

  bool reloadConfigCb(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
  void adcCb(const tobas_hal_msgs::AdcConstPtr& adc);
};
}  // namespace tobas_real_ros

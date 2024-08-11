#pragma once

#include <std_srvs/srv/trigger.hpp>

#include <tobas_property_tools/property_client.hpp>
#include <tobas_node/node.hpp>
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
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Config
  double voltage_coef_ = kDefaultAdcVoltageCoef;
  double current_coef_ = kDefaultAdcCurrentCoef;

  ptree::PropertyClient property_client_;

  PublisherPtr<tobas_msgs::msg::Battery> battery_pub_;
  SubscriberPtr<> adc_sub_;

  ServicePtr<> reload_config_srv_;

  bool reloadConfig();

  bool reloadConfigCb(std_srvs::srv::Trigger::Request& req, std_srvs::srv::Trigger::Response& res);
  void adcCb(const tobas_hal_msgs::Adc::ConstSharedPtr& adc);
};
}  // namespace tobas_real_ros

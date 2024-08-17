#include <tobas_constants/constants.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_msgs/msg/battery.hpp>

#include "../include/tobas_real_ros/battery_handler.hpp"
#include "../include/tobas_real_ros/common.hpp"

using namespace std;

namespace tobas_real_ros
{
BatteryHandler::BatteryHandler(const rclcpp::NodeOptions& options)
  : super(name, options), property_client_(node_, kPropertyServerFC)
{
  reloadConfig();

  battery_pub_ = createPublisher<tobas_msgs::msg::Battery>(tobas::kBatteryTopic);
  adc_sub_ = createSubscriber(hal::kAdcTopic, &self::adcCb, this);

  reload_config_srv_ = createService(name + tobas::kReloadConfigSrvSuffix, &self::reloadConfigCb, this);
}

bool BatteryHandler::reloadConfig()
{
  if (property_client_.get(kConfigKey_AdcVoltageCoef, voltage_coef_) < 0)
  {
    TOBAS_ERROR(property_client_.errorMessage());
    return false;
  }

  // TODO: 電流の係数も取得

  return true;
}

bool BatteryHandler::reloadConfigCb(std_srvs::srv::Trigger::Request&, std_srvs::srv::Trigger::Response& res)
{
  if (!reloadConfig())
  {
    res.success = false;
    res.message = "Failed to reload configurations.";
    return true;
  }

  res.success = true;
  return true;
}

void BatteryHandler::adcCb(const tobas_hal_msgs::Adc::ConstSharedPtr& adc)
{
  // Create battery message
  const auto battery_msg =std::make_unique<tobas_msgs::msg::Battery>();
  battery_msg->header = adc->header;

  // Fill values
  battery_msg->voltage = adc->voltage * voltage_coef_;
  battery_msg->current = adc->current * current_coef_;

  // Publish battery message
  battery_pub_->publish(battery_msg);
}
}  // namespace tobas_real_ros

#include <std_srvs/srv/trigger.hpp>

#include <tobas_node/node.hpp>
#include <tobas_property_tools/property_client.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/msg/adc.hpp>
#include <tobas_msgs/msg/battery.hpp>

#include "../include/tobas_real_ros/common.hpp"

using namespace std;

class BatteryHandlerNode : public tobas::BaseNode
{
  // Defaults (cf. ADC example: https://docs.emlid.com/navio2/dev/adc)
  static constexpr double kDefaultAdcVoltageCoef = 11.3;
  static constexpr double kDefaultAdcCurrentCoef = 17.0;

  using self = BatteryHandlerNode;
  using super = tobas::BaseNode;

public:
  explicit BatteryHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Config
  double voltage_coef_ = kDefaultAdcVoltageCoef;
  double current_coef_ = kDefaultAdcCurrentCoef;

  ptree::PropertyClient::SharedPtr property_client_;

  PublisherPtr<tobas_msgs::msg::Battery> battery_pub_;
  SubscriberPtr<tobas_hal_msgs::msg::Adc> adc_sub_;

  ServicePtr<std_srvs::srv::Trigger> reload_config_srv_;

  TimerPtr initialize_timer_;
  void initializeTimerCb();

  bool reloadConfig();

  void adcCb(const tobas_hal_msgs::msg::Adc::ConstSharedPtr& adc);
  void reloadConfigCb(
    const std_srvs::srv::Trigger::Request::ConstSharedPtr& req,
    const std_srvs::srv::Trigger::Response::SharedPtr& res);
};

BatteryHandlerNode::BatteryHandlerNode(const rclcpp::NodeOptions& options) : super("battery_handler", options)
{
  initialize_timer_ = createTimer(0ns, &self::initializeTimerCb, this);
}

void BatteryHandlerNode::initializeTimerCb()
{
  property_client_ = std::make_shared<ptree::PropertyClient>(shared_from_this(), real::kPropertyServerFC);
  reloadConfig();

  battery_pub_ = createPublisher<tobas_msgs::msg::Battery>(tobas::kBatteryTopic);
  adc_sub_ = createSubscriber(hal::kAdcTopic, &self::adcCb, this);

  reload_config_srv_ =
    createService<std_srvs::srv::Trigger>(name() + tobas::kReloadConfigSrvSuffix, &self::reloadConfigCb, this);

  initialize_timer_->cancel();
}

bool BatteryHandlerNode::reloadConfig()
{
  if (property_client_->get(real::kConfigKey_AdcVoltageCoef, voltage_coef_) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    return false;
  }

  // TODO: 電流の係数も取得

  return true;
}

void BatteryHandlerNode::adcCb(const tobas_hal_msgs::msg::Adc::ConstSharedPtr& adc)
{
  // Create battery message
  auto battery_msg = std::make_unique<tobas_msgs::msg::Battery>();
  battery_msg->header = adc->header;

  // Fill values
  battery_msg->voltage = adc->voltage * voltage_coef_;
  battery_msg->current = adc->current * current_coef_;

  // Publish battery message
  battery_pub_->publish(move(battery_msg));
}

void BatteryHandlerNode::reloadConfigCb(
  const std_srvs::srv::Trigger::Request::ConstSharedPtr&,
  const std_srvs::srv::Trigger::Response::SharedPtr& res)
{
  if (!reloadConfig())
  {
    res->success = false;
    res->message = "Failed to reload configurations.";
    return;
  }

  res->success = true;
  res->message.clear();
}

RCLCPP_COMPONENTS_REGISTER_NODE(BatteryHandlerNode)

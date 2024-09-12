#include <std_srvs/srv/trigger.hpp>

#include <tobas_property_tree/property_tree.hpp>
#include <tobas_linux/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/msg/adc.hpp>
#include <tobas_msgs/msg/battery.hpp>

#include <tobas_real_common/constants.hpp>

using namespace std;
using namespace real::handler::adc;

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

  ptree::PropertyTree pt_;

  ros2::PublisherPtr<tobas_msgs::msg::Battery> battery_pub_;
  ros2::SubscriberPtr<tobas_hal_msgs::msg::Adc> adc_sub_;

  void readConfig();

  bool paramsCb(const std::vector<double>& params);
  void adcCb(const tobas_hal_msgs::msg::Adc::ConstSharedPtr& adc);
};

BatteryHandlerNode::BatteryHandlerNode(const rclcpp::NodeOptions& options) : super("battery_handler", options)
{
  if (!pt_.initialize(linux::expandUser(kIniPath)))
    TOBAS_EXIT("Failed to initialize property tree.");

  readConfig();

  addDynamicDoubleArrayParam(real::handler::kParamName, &self::paramsCb, this);

  battery_pub_ = createPublisher<tobas_msgs::msg::Battery>(tobas::kBatteryTopic);
  adc_sub_ = createSubscriber(hal::kAdcTopic, &self::adcCb, this);
}

void BatteryHandlerNode::readConfig()
{
  if (!pt_.get(kVoltageKey, voltage_coef_))
  {
    TOBAS_WARN("Failed to get \"", kVoltageKey, "\". from configuration file. The default value is used.");
    voltage_coef_ = kDefaultAdcVoltageCoef;
  }

  if (!pt_.get(kCurrentKey, voltage_coef_))
  {
    TOBAS_WARN("Failed to get \"", kCurrentKey, "\". from configuration file. The default value is used.");
    current_coef_ = kDefaultAdcCurrentCoef;
  }
}

bool BatteryHandlerNode::paramsCb(const std::vector<double>& params)
{
  // Skip first call
  if (params.size() == 0)
    return false;

  // Check size
  if (params.size() != kParamSize)
  {
    TOBAS_ERROR("Parameter size mismatch.");
    return false;
  }

  // Verify parameters
  if (params.at(kVoltageChannel) <= 0.)
  {
    TOBAS_ERROR("Voltage coefficient must be positive.");
    return false;
  }
  if (params.at(kCurrentChannel) <= 0.)
  {
    TOBAS_ERROR("Current coefficient must be positive.");
    return false;
  }

  // Update parameters
  voltage_coef_ = params.at(kVoltageChannel);
  current_coef_ = params.at(kCurrentChannel);

  // Save parameters
  pt_.set(kVoltageKey, params.at(kVoltageChannel));
  pt_.set(kCurrentKey, params.at(kCurrentChannel));
  if (pt_.save())
  {
    TOBAS_ERROR("Failed to save parameters.");
    return false;
  }

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

RCLCPP_COMPONENTS_REGISTER_NODE(BatteryHandlerNode)

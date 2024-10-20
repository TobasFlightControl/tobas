#include <tobas_hal_core/base_sensor_node.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/msg/adc.hpp>

#include <tobas_aso_core/ads1220.hpp>

using namespace std;

class ADCDriverNode : public hal::BaseSensorNode
{
  static constexpr auto kSamplingPeriod = 10ms;

  using self = ADCDriverNode;
  using super = hal::BaseSensorNode;

public:
  explicit ADCDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  aso::ADS1220 adc_;
  ros2::PublisherPtr<tobas_hal_msgs::msg::Adc> adc_pub_;

  void mainTimerCb();
};

ADCDriverNode::ADCDriverNode(const rclcpp::NodeOptions& options) : super("aso_adc_driver", options)
{
  if (!adc_.initialize())
    TOBAS_EXIT("Failed to initialize ADC.");

  adc_pub_ = createPublisher<tobas_hal_msgs::msg::Adc>(hal::kADCTopic);
  main_timer_ = createTimer(kSamplingPeriod, &self::mainTimerCb, this);
}

void ADCDriverNode::mainTimerCb()
{
  // Create messages
  auto msg = std::make_unique<tobas_hal_msgs::msg::Adc>();

  // Fill headers
  msg->header.stamp = get_clock()->now();

  // Read ADC
  if (!adc_.readVoltage(msg->voltage))
  {
    TOBAS_FATAL("Failed to read voltage.");
    return;
  }
  msg->current = 0.;  // TODO

  // Publish message
  adc_pub_->publish(move(msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(ADCDriverNode)

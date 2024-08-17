#include <tobas_hal_core/base_sensor_node.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/msg/fluid_pressure.hpp>

#include <tobas_a1_core/ilps22qs.hpp>

using namespace std;

class BaroDriverNode : public hal::BaseSensorNode
{
  static constexpr auto kSamplingPeriod = 10ms;

  using self = BaroDriverNode;
  using super = hal::BaseSensorNode;

public:
  explicit BaroDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  a1::ILPS22QS baro_;
  PublisherPtr<tobas_hal_msgs::msg::FluidPressure> baro_pub_;

  void mainTimerCb();
};

BaroDriverNode::BaroDriverNode(const rclcpp::NodeOptions& options) : super("a1_baro_driver", options)
{
  if (!baro_.initialize())
    TOBAS_EXIT("Failed to initialize Barometer.");

  baro_pub_ = createPublisher<tobas_hal_msgs::msg::FluidPressure>(hal::kAirPressureTopic);
  main_timer_ = createTimer(kSamplingPeriod, &self::mainTimerCb, this);
}

void BaroDriverNode::mainTimerCb()
{
  // Create messages
  auto msg = std::make_unique<tobas_hal_msgs::msg::FluidPressure>();

  // Fill headers
  msg->header.stamp = get_clock()->now();

  // Read sensor
  if (!baro_.readPressure(msg->fluid_pressure))
  {
    TOBAS_FATAL("Failed to read barometer.");
    return;
  }

  // Publish message
  baro_pub_->publish(move(msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(BaroDriverNode)

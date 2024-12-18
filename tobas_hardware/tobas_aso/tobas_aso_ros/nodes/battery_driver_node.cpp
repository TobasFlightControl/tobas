#include <tobas_constants/constants.hpp>
#include <tobas_hardware_common/base_sensor_node.hpp>
#include <tobas_msgs/msg/battery.hpp>

#include <tobas_aso_core/battery.hpp>

using namespace std;

class BatteryDriverNode : public hardware::BaseSensorNode
{
  static constexpr auto kSamplingPeriod = 10ms;

  using self = BatteryDriverNode;
  using super = hardware::BaseSensorNode;

public:
  explicit BatteryDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  aso::Battery battery_;
  ros2::PublisherPtr<tobas_msgs::msg::Battery> battery_pub_;

  void mainTimerCb();
};

BatteryDriverNode::BatteryDriverNode(const rclcpp::NodeOptions& options) : super("aso_battery_driver", options)
{
  if (!battery_.initialize())
    TOBAS_EXIT("Failed to initialize battery driver.");

  battery_pub_ = createPublisher<tobas_msgs::msg::Battery>(tobas::kBatteryTopic);
  main_timer_ = createTimer(kSamplingPeriod, &self::mainTimerCb, this);
}

void BatteryDriverNode::mainTimerCb()
{
  // Create messages
  auto msg = std::make_unique<tobas_msgs::msg::Battery>();

  // Fill headers
  msg->header.stamp = get_clock()->now();

  // Read data
  if (!battery_.read(msg->voltage, msg->current))
  {
    TOBAS_FATAL("Failed to read battery state.");
    return;
  }

  // Publish message
  battery_pub_->publish(move(msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(BatteryDriverNode)

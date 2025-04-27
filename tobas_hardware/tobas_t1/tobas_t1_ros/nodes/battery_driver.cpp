#include <tobas_constants/constants.hpp>
#include <tobas_hardware_common/base_sensor_node.hpp>
#include <tobas_msgs/msg/battery.hpp>

#include <tobas_t1_core/battery.hpp>

#include "./common.hpp"

using namespace std;

class BatteryDriverNode : public hardware::BaseSensorNode
{
  static constexpr auto kSamplingPeriod = 1000ms;  // TODO: SPIデバイスをうまく分離してもっと上げる

  using self = BatteryDriverNode;
  using super = hardware::BaseSensorNode;

public:
  explicit BatteryDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  t1::Battery battery_;
  ros2::PublisherPtr<tobas_msgs::msg::Battery> battery_pub_;
  ros2::TimerPtr initialize_timer_;

  void initialize();
  void mainTimerCb();
};

BatteryDriverNode::BatteryDriverNode(const rclcpp::NodeOptions& options) : super("t1_battery_driver", options)
{
  initialize_timer_ = createWallTimer(t1::kRetryInitializationInterval, &self::initialize, this);
}

void BatteryDriverNode::initialize()
{
  if (!battery_.initialize())
  {
    TOBAS_ERROR("Failed to initialize battery driver. Retrying...");
    return;
  }

  battery_pub_ = createPublisher<tobas_msgs::msg::Battery>(tobas::kBatteryTopic);

  initialize_timer_->cancel();
  main_timer_ = createWallTimer(kSamplingPeriod, &self::mainTimerCb, this);
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

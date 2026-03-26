#include <tobas_constants/ros_interface.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs/msg/battery.hpp>

using namespace std::chrono_literals;

namespace tobas
{
class FakeBattPublisherNode : public BaseNode
{
  static constexpr auto kSamplingPeriod = 10ms;

  static constexpr double kDefaultVoltage = 14.8;  // [V]
  static constexpr double kDefaultCurrent = 20.;   // [A]

  using self = FakeBattPublisherNode;
  using super = BaseNode;

public:
  explicit FakeBattPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  double voltage_;
  double current_;

  ros2::PublisherPtr<tobas_msgs::msg::Battery> batt_pub_;
  ros2::TimerPtr timer_;

  void timerCb();
};

FakeBattPublisherNode::FakeBattPublisherNode(const rclcpp::NodeOptions& options)
  : super("fake_batt_publisher", nodeOptions_Default(options))
{
  voltage_ = getDoubleParam("voltage", kDefaultVoltage);
  current_ = getDoubleParam("current", kDefaultCurrent);

  batt_pub_ = createPublisher<tobas_msgs::msg::Battery>(topic::kBattery);
  timer_ = createTimer(kSamplingPeriod, &self::timerCb, this);
}

void FakeBattPublisherNode::timerCb()
{
  auto batt_msg = std::make_unique<tobas_msgs::msg::Battery>();
  batt_msg->header.stamp = now();
  batt_msg->voltage = voltage_;
  batt_msg->current = current_;

  batt_pub_->publish(std::move(batt_msg));
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::FakeBattPublisherNode)

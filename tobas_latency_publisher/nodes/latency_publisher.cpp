#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/latency.hpp>

class LatencyPublisherNode : public tobas::BaseNode
{
  using self = LatencyPublisherNode;
  using super = tobas::BaseNode;

public:
  explicit LatencyPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ros2::PublisherPtr<tobas_msgs::msg::Latency> latency_pub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorSpeedArray> tar_speeds_sub_;

  void targetSpeedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& msg);
};

LatencyPublisherNode::LatencyPublisherNode(const rclcpp::NodeOptions& options) : super("latency_publisher", options)
{
  latency_pub_ = createPublisher<tobas_msgs::msg::Latency>(tobas::kLatencyTopic);
  tar_speeds_sub_ = createSubscriber(tobas::kRotorSpeedsCmdTopic, &self::targetSpeedsCb, this);
}

void LatencyPublisherNode::targetSpeedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& msg)
{
  const auto cur_time = get_clock()->now();

  auto latency = std::make_unique<tobas_msgs::msg::Latency>();
  latency->header.stamp = cur_time;
  latency->data = cur_time - msg->header.stamp;

  latency_pub_->publish(std::move(latency));
}

RCLCPP_COMPONENTS_REGISTER_NODE(LatencyPublisherNode)

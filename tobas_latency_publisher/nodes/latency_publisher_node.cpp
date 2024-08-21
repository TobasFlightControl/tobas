#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/throttle_array.hpp>
#include <tobas_msgs/msg/latency.hpp>

using namespace std;

class LatencyPublisherNode : public tobas::BaseNode
{
  using self = LatencyPublisherNode;
  using super = tobas::BaseNode;

public:
  explicit LatencyPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ros2::PublisherPtr<tobas_msgs::msg::Latency> latency_pub_;
  ros2::SubscriberPtr<tobas_msgs::msg::ThrottleArray> throttles_sub_;

  void throttlesCb(const tobas_msgs::msg::ThrottleArray::ConstSharedPtr& msg);
};

LatencyPublisherNode::LatencyPublisherNode(const rclcpp::NodeOptions& options) : super("latency_publisher", options)
{
  latency_pub_ = createPublisher<tobas_msgs::msg::Latency>(tobas::kLatencyTopic);
  throttles_sub_ = createSubscriber(tobas::kThrottlesCmdTopic, &self::throttlesCb, this);
}

void LatencyPublisherNode::throttlesCb(const tobas_msgs::msg::ThrottleArray::ConstSharedPtr& msg)
{
  auto latency = std::make_unique<tobas_msgs::msg::Latency>();
  const auto cur_time = get_clock()->now();
  latency->header.stamp = cur_time;
  latency->data = cur_time - msg->header.stamp;
  latency_pub_->publish(move(latency));
}

RCLCPP_COMPONENTS_REGISTER_NODE(LatencyPublisherNode)

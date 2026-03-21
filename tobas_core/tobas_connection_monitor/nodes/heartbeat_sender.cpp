#include <tobas_constants/ros_interface.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs/msg/heartbeat.hpp>

using namespace std::chrono_literals;

class HeartbeatSenderNode : public tobas::BaseNode
{
  static constexpr auto kPublishPeriod = 1s;

  using self = HeartbeatSenderNode;
  using super = tobas::BaseNode;

public:
  explicit HeartbeatSenderNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ros2::PublisherPtr<tobas_msgs::msg::Heartbeat> heartbeat_pub_;
  ros2::TimerPtr main_timer_;

  void mainTimerCb();
};

HeartbeatSenderNode::HeartbeatSenderNode(const rclcpp::NodeOptions& options)
  : super("heartbeat_sender", nodeOptions_Default(options))
{
  heartbeat_pub_ = createPublisher<tobas_msgs::msg::Heartbeat>(tobas::topic::kHeartbeat);
  main_timer_ = createTimer(kPublishPeriod, &self::mainTimerCb, this);
}

void HeartbeatSenderNode::mainTimerCb()
{
  auto heartbeat = std::make_unique<tobas_msgs::msg::Heartbeat>();
  heartbeat_pub_->publish(std::move(heartbeat));
}

RCLCPP_COMPONENTS_REGISTER_NODE(HeartbeatSenderNode)

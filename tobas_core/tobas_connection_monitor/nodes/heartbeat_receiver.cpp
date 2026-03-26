#include <tobas_constants/ros_interface.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs/msg/heartbeat.hpp>
#include <tobas_msgs/msg/remote_connection.hpp>

using namespace std::chrono_literals;

namespace tobas
{
class HeartbeatReceiverNode : public BaseNode
{
  static constexpr auto kConnectionTimeout = 5s;

  using self = HeartbeatReceiverNode;
  using super = BaseNode;

public:
  explicit HeartbeatReceiverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ros2::PublisherPtr<tobas_msgs::msg::RemoteConnection> connection_pub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Heartbeat> heartbeat_sub_;
  ros2::TimerPtr timeout_timer_;

  void publishConnectionState(bool connected);

  void heartbeatCb(const tobas_msgs::msg::Heartbeat::ConstSharedPtr& msg);
  void onConnectionTimeout();
};

HeartbeatReceiverNode::HeartbeatReceiverNode(const rclcpp::NodeOptions& options)
  : super("heartbeat_receiver", nodeOptions_Default(options))
{
  connection_pub_ = createPublisher<tobas_msgs::msg::RemoteConnection>(topic::kRemoteConnection);
  heartbeat_sub_ = createSubscriber(topic::kHeartbeat, &self::heartbeatCb, this);
  timeout_timer_ = createTimer(kConnectionTimeout, &self::onConnectionTimeout, this);
}

void HeartbeatReceiverNode::publishConnectionState(bool connected)
{
  auto msg = std::make_unique<tobas_msgs::msg::RemoteConnection>();
  msg->header.stamp = now();
  msg->data = connected;
  connection_pub_->publish(std::move(msg));
}

void HeartbeatReceiverNode::heartbeatCb(const tobas_msgs::msg::Heartbeat::ConstSharedPtr&)
{
  timeout_timer_->reset();
  publishConnectionState(true);
}

void HeartbeatReceiverNode::onConnectionTimeout()
{
  publishConnectionState(false);
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::HeartbeatReceiverNode)

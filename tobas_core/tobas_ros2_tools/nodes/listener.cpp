#include "./listener.hpp"

namespace ros2
{
Listener::Listener(const rclcpp::NodeOptions&) : super("listener", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  sub_ = createSubscriber<std_msgs::msg::String>("chatter", 1, &self::msgCb, this);
}

void Listener::msgCb(const std_msgs::msg::String::ConstSharedPtr& msg)
{
  TOBAS_INFO("I heard: ", msg->data);
}
}  // namespace ros2

RCLCPP_COMPONENTS_REGISTER_NODE(ros2::Listener)

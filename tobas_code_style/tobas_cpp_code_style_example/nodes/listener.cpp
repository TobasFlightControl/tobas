#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <std_msgs/msg/string.hpp>

namespace tobas
{
class Listener : public rclcpp::Node
{
public:
  explicit Listener(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());

private:
  void msgCb(const std_msgs::msg::String::ConstSharedPtr& _msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

Listener::Listener(const rclcpp::NodeOptions& _options) : rclcpp::Node("listener", _options)
{
  sub_ =
    create_subscription<std_msgs::msg::String>("chatter", 1, std::bind(&Listener::msgCb, this, std::placeholders::_1));
}

void Listener::msgCb(const std_msgs::msg::String::ConstSharedPtr& _msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "I heard: \"" << _msg->data << "\" (" << _msg.get() << ")");
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::Listener)

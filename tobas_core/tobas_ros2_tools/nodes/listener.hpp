#include <std_msgs/msg/string.hpp>

#include "../include/tobas_ros2_tools/node.hpp"

namespace ros2
{
class Listener : public Node
{
  using self = Listener;
  using super = Node;

public:
  explicit Listener(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  size_t cnt_ = 0;
  SubscriberPtr<std_msgs::msg::String> sub_;

  void msgCb(const std_msgs::msg::String::ConstSharedPtr& msg);
};
}  // namespace ros2

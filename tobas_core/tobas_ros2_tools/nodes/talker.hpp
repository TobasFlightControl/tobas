#include <std_msgs/msg/string.hpp>

#include "../include/tobas_ros2_tools/node.hpp"

namespace ros2
{
class Talker : public Node
{
  using self = Talker;
  using super = Node;

public:
  explicit Talker(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  size_t cnt_ = 0;
  PublisherPtr<std_msgs::msg::String> pub_;
  TimerPtr timer_;

  void timerCb();
};
}  // namespace ros2

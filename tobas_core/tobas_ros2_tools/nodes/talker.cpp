#include "./talker.hpp"

using namespace std;

namespace ros2
{
Talker::Talker(const rclcpp::NodeOptions& options) : super("talker", options)
{
  pub_ = createPublisher<std_msgs::msg::String>("chatter", 1);
  timer_ = createTimer(1s, &self::timerCb, this);
}

void Talker::timerCb()
{
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = "Hello World: " + to_string(cnt_++);
  TOBAS_INFO("Publishing: \"", msg->data, "\" (", &(msg->data), ")");
  pub_->publish(move(msg));
}
}  // namespace ros2

RCLCPP_COMPONENTS_REGISTER_NODE(ros2::Talker)

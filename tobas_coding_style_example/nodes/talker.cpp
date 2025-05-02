#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace tobas
{
class Talker : public rclcpp::Node
{
public:
  explicit Talker(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());

private:
  void timerCb();

  size_t cnt_ = 0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

Talker::Talker(const rclcpp::NodeOptions& _options) : rclcpp::Node("talker", _options)
{
  pub_ = create_publisher<std_msgs::msg::String>("chatter", 1);
  timer_ = create_timer(1s, bind(&Talker::timerCb, this));
}

void Talker::timerCb()
{
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = "Hello World: " + std::to_string(cnt_++);
  RCLCPP_INFO_STREAM(get_logger(), "Publishing: \"" << msg->data << "\" (" << msg.get() << ")");
  pub_->publish(move(msg));
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::Talker)

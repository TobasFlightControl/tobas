#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/cpu.hpp>

using namespace std;

class CpuHandlerNode : public tobas::BaseNode
{
  static constexpr auto kSamplingPeriod = 1s;

  using self = CpuHandlerNode;
  using super = tobas::BaseNode;

public:
  explicit CpuHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Publisher
  ros2::PublisherPtr<tobas_msgs::msg::Cpu> cpu_pub_;

  // Timer
  ros2::TimerPtr main_timer_;

  void mainTimerCb();
};

CpuHandlerNode::CpuHandlerNode(const rclcpp::NodeOptions& options) : super("cpu_handler", options)
{
  cpu_pub_ = createPublisher<tobas_msgs::msg::Cpu>(tobas::kCpuTopic);
  main_timer_ = createTimer(kSamplingPeriod, &self::mainTimerCb, this);
}

void CpuHandlerNode::mainTimerCb()
{
  // Create ROS message
  auto cpu_msg = std::make_unique<tobas_msgs::msg::Cpu>();
  cpu_msg->header.stamp = get_clock()->now();

  // TODO: Get CPU information in a cross-platform way

  // Publish ROS message
  cpu_pub_->publish(move(cpu_msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(CpuHandlerNode)

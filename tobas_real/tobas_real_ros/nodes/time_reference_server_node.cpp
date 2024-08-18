#include <sensor_msgs/msg/time_reference.hpp>

#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>

using namespace std;

class TimeReferenceServerNode : public tobas::BaseNode
{
  static constexpr auto kUpdatePeriod = 1s;

  using self = TimeReferenceServerNode;
  using super = tobas::BaseNode;

public:
  explicit TimeReferenceServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  PublisherPtr<sensor_msgs::msg::TimeReference> time_ref_pub_;
  TimerPtr main_timer_;

  void mainTimerCb();
};

TimeReferenceServerNode::TimeReferenceServerNode(const rclcpp::NodeOptions& options)
  : super("time_reference_server", options)
{
  time_ref_pub_ = createPublisher<sensor_msgs::msg::TimeReference>(tobas::kTimeReferenceTopic);
  main_timer_ = createTimer(kUpdatePeriod, &self::mainTimerCb, this);
}

void TimeReferenceServerNode::mainTimerCb()
{
  auto time_ref = std::make_unique<sensor_msgs::msg::TimeReference>();
  const auto now = get_clock()->now();
  time_ref->header.stamp = now;
  time_ref->time_ref = now;
  time_ref_pub_->publish(move(time_ref));
}

RCLCPP_COMPONENTS_REGISTER_NODE(TimeReferenceServerNode)

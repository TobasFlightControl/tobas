#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/latency.hpp>

using namespace std;

class LatencyCheckerNode : public tobas::BaseNode
{
  static constexpr long kLatencyWarnThresh = 2000;  // [us]

  using self = LatencyCheckerNode;
  using super = tobas::BaseNode;

public:
  explicit LatencyCheckerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ros2::SubscriberPtr<tobas_msgs::msg::Latency> latency_sub_;

  void latencyCb(const tobas_msgs::msg::Latency::ConstSharedPtr& latency);
};

LatencyCheckerNode::LatencyCheckerNode(const rclcpp::NodeOptions& options) : super("latency_checker", options)
{
  latency_sub_ = createSubscriber(tobas::kLatencyTopic, &self::latencyCb, this);
}

void LatencyCheckerNode::latencyCb(const tobas_msgs::msg::Latency::ConstSharedPtr& latency)
{
  // オーバーフロー回避のためlongにキャスト
  const auto sec = static_cast<long>(latency->data.sec);
  const auto nsec = static_cast<long>(latency->data.nanosec);

  // 遅延をマイクロ秒に変換
  const auto latency_us = sec * 1'000'000 + nsec / 1'000;

  // 遅延が閾値を超えていたら警告
  if (latency_us > kLatencyWarnThresh)
  {
    TOBAS_WARN_THROTTLE(
      tobas::kTypicalWarnPeriod, "Control latency is too large: ", latency_us, " > ", kLatencyWarnThresh, " [us]");
  }

  // TODO: 遅延があまりに大きい場合はパラシュートを開くなどの措置
}

RCLCPP_COMPONENTS_REGISTER_NODE(LatencyCheckerNode)

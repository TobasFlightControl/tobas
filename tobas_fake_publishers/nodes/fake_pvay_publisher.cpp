#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs_adapter/PosVelAccYaw.hpp>

using namespace std;

class FakePosVelAccYawPublisherNode : public tobas::BaseNode
{
  static constexpr auto kSamplingPeriod = 10ms;

  using self = FakePosVelAccYawPublisherNode;
  using super = tobas::BaseNode;

public:
  explicit FakePosVelAccYawPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  double pos_stddev_;
  double vel_stddev_;

  ros2::PublisherPtr<tobas_msgs::PosVelAccYaw> pvay_pub_;
  ros2::TimerPtr timer_;

  void timerCb();
};

FakePosVelAccYawPublisherNode::FakePosVelAccYawPublisherNode(const rclcpp::NodeOptions& options)
  : super("fake_battery_publisher", options)
{
  pvay_pub_ = createPublisher<tobas_msgs::PosVelAccYaw>(tobas::kPosVelAccYawCmdTopic);
  timer_ = createTimer(kSamplingPeriod, &self::timerCb, this);
}

void FakePosVelAccYawPublisherNode::timerCb()
{
  auto pvay_msg = std::make_unique<tobas_msgs::PosVelAccYaw>();
  pvay_msg->pos.setZero();
  pvay_msg->vel.setZero();
  pvay_msg->acc.setZero();
  pvay_msg->yaw = 0.;

  pvay_pub_->publish(move(pvay_msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(FakePosVelAccYawPublisherNode)

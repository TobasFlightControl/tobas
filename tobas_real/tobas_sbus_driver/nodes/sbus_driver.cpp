#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/sbus.hpp>

#include <tobas_sbus_driver/sbus.hpp>
#include <tobas_aso_core/constants.hpp>

using namespace std;

class SBUSDriverNode : public tobas::BaseNode
{
  using self = SBUSDriverNode;
  using super = tobas::BaseNode;

public:
  explicit SBUSDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  string device_;
  tobas::SBUS sbus_;
  ros2::PublisherPtr<tobas_msgs::msg::Sbus> sbus_pub_;
  ros2::TimerPtr initialize_timer_;

  void initialize();
  void onPacket(const tobas::SBUS::Packet& packet);
};

SBUSDriverNode::SBUSDriverNode(const rclcpp::NodeOptions& options)
  : super("sbus_driver", options), sbus_(bind(&self::onPacket, this, placeholders::_1))
{
  device_ = getStringParam("device");

  sbus_pub_ = createPublisher<tobas_msgs::msg::Sbus>(tobas::kSBUSTopic);

  initialize_timer_ = createTimer(3s, &self::initialize, this);
}

void SBUSDriverNode::initialize()
{
  if (!sbus_.initialize(device_.c_str()))
  {
    TOBAS_WARN("Failed to initialize S.BUS driver with device \"", device_, "\". Retrying...");
    return;
  }

  initialize_timer_->cancel();
  sbus_.start();
}

void SBUSDriverNode::onPacket(const tobas::SBUS::Packet& packet)
{
  // Create message
  auto sbus_msg = std::make_unique<tobas_msgs::msg::Sbus>();
  sbus_msg->header.stamp = get_clock()->now();
  sbus_msg->data = packet.periods;

  // Publish message
  sbus_pub_->publish(move(sbus_msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(SBUSDriverNode)

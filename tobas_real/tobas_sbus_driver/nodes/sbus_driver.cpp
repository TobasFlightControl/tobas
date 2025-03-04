#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/sbus.hpp>

#include <tobas_sbus_driver/sbus.hpp>
#include <tobas_aso_core/constants.hpp>

class SBUSDriverNode : public tobas::BaseNode
{
  using self = SBUSDriverNode;
  using super = tobas::BaseNode;

public:
  explicit SBUSDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::SBUS sbus_;

  ros2::PublisherPtr<tobas_msgs::msg::Sbus> sbus_pub_;

  void onPacket(const tobas::SBUS::Packet& packet);
};

SBUSDriverNode::SBUSDriverNode(const rclcpp::NodeOptions& options)
  : super("sbus_driver", options), sbus_(std::bind(&self::onPacket, this, std::placeholders::_1))
{
  const auto device = getStringParam("device");

  // Initialize SBUS driver
  if (!sbus_.initialize(device.c_str()))
    TOBAS_EXIT("Failed to initialize S.BUS driver.");

  // Advertise publisher
  sbus_pub_ = createPublisher<tobas_msgs::msg::Sbus>(tobas::kSBUSTopic);

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

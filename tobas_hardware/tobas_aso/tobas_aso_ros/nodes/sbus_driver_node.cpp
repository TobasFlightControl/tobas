#include <tobas_constants/constants.hpp>
#include <tobas_hardware_common/base_sensor_node.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_msgs/msg/sbus.hpp>

#include <tobas_ic_drivers/sbus.hpp>
#include <tobas_aso_core/constants.hpp>

using namespace std;

class SBUSDriverNode : public hardware::BaseSensorNode
{
  using self = SBUSDriverNode;
  using super = hardware::BaseSensorNode;

public:
  explicit SBUSDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  driver::SBUS sbus_;

  ros2::PublisherPtr<tobas_msgs::msg::Sbus> sbus_pub_;

  void onPacket(const driver::SBUS::Packet& packet);
};

SBUSDriverNode::SBUSDriverNode(const rclcpp::NodeOptions& options)
  : super("aso_sbus_driver", options), sbus_(bind(&self::onPacket, this, placeholders::_1))
{
  // Advertise publisher
  sbus_pub_ = createPublisher<tobas_msgs::msg::Sbus>(real::kSBUSTopic);

  // Initialize SBUS driver
  if (!sbus_.initialize(aso::uart_device::kSbusDev))
    TOBAS_EXIT("Failed to initialize S.BUS driver.");

  sbus_.start();
}

void SBUSDriverNode::onPacket(const driver::SBUS::Packet& packet)
{
  // Create message
  auto sbus_msg = std::make_unique<tobas_msgs::msg::Sbus>();
  sbus_msg->header.stamp = get_clock()->now();
  sbus_msg->data = packet.periods;

  // Publish message
  sbus_pub_->publish(move(sbus_msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(SBUSDriverNode)

#include <tobas_constants/constants.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs/msg/sbus.hpp>

#include "tobas_sbus_driver/sbus.hpp"

using namespace std::chrono_literals;

class SbusDriverNode : public tobas::BaseNode
{
  using self = SbusDriverNode;
  using super = tobas::BaseNode;

public:
  explicit SbusDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  std::string device_;
  tobas::SBUS sbus_;
  ros2::PublisherPtr<tobas_msgs::msg::Sbus> sbus_pub_;
  ros2::TimerPtr initialize_timer_;

  void initialize();
  void onPacket(const tobas::SBUS::Packet& packet);
};

SbusDriverNode::SbusDriverNode(const rclcpp::NodeOptions& options)
  : super("sbus_driver", options), sbus_(std::bind(&self::onPacket, this, std::placeholders::_1))
{
  device_ = getStringParam("device", "");
  if (device_.empty()) {
    TOBAS_WARN("No device name specified. This node will not work.");
    return;
  }

  sbus_pub_ = createPublisher<tobas_msgs::msg::Sbus>(tobas::kSbusTopic);

  initialize_timer_ = createWallTimer(3s, &self::initialize, this);
}

void SbusDriverNode::initialize()
{
  if (!sbus_.initialize(device_.c_str())) {
    TOBAS_WARN("Failed to initialize S.BUS driver with device \"", device_, "\". Retrying...");
    return;
  }

  initialize_timer_->cancel();

  sbus_.start();
}

void SbusDriverNode::onPacket(const tobas::SBUS::Packet& packet)
{
  // Create message
  auto sbus_msg = std::make_unique<tobas_msgs::msg::Sbus>();
  sbus_msg->header.stamp = now();

  sbus_msg->periods = packet.periods;
  sbus_msg->ch17 = packet.ch17;
  sbus_msg->ch18 = packet.ch18;
  sbus_msg->frame_lost = packet.frame_lost;
  sbus_msg->failsafe = packet.failsafe;

  // Publish message
  sbus_pub_->publish(std::move(sbus_msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(SbusDriverNode)

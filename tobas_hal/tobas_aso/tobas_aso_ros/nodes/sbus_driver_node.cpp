#include <SBUS.h>

#include <tobas_constants/constants.hpp>
#include <tobas_hal_core/base_sensor_node.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/msg/sbus.hpp>

#include <tobas_aso_core/constants.hpp>

using namespace std;

class SBUSDriverNode : public hal::BaseSensorNode
{
  using self = SBUSDriverNode;
  using super = hal::BaseSensorNode;

public:
  explicit SBUSDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  SBUS sbus_;
  sbus_err_t error_;

  ros2::PublisherPtr<tobas_hal_msgs::msg::Sbus> sbus_pub_;

  void mainTimerCb();
};

SBUSDriverNode::SBUSDriverNode(const rclcpp::NodeOptions& options) : super("aso_sbus_driver", options)
{
  // Initialize S.BUS driver with blocking mode
  if ((error_ = sbus_.install(aso::uart_device::kSbusDev, true)) != SBUS_OK)
    TOBAS_EXIT("Failed to install S.BUS with error code ", error_, ".");

  // Advertise publisher
  sbus_pub_ = createPublisher<tobas_hal_msgs::msg::Sbus>(hal::kSbusTopic);

  // Start main timer with maximum rate
  main_timer_ = createTimer(0ns, &self::mainTimerCb, this);
}

void SBUSDriverNode::mainTimerCb()
{
  // Read S.BUS
  if ((error_ = sbus_.read()) != SBUS_OK)
  {
    TOBAS_ERROR_THROTTLE(tobas::kTypicalErrorPeriod, "Failed to read S.BUS with error code ", error_, ".");
    return;
  }

  // Create message
  const auto& packet = sbus_.lastPacket();
  auto sbus_msg = std::make_unique<tobas_hal_msgs::msg::Sbus>();
  sbus_msg->header.stamp = get_clock()->now();
  copy(begin(packet.channels), end(packet.channels), sbus_msg->data.begin());

  // Publish message
  sbus_pub_->publish(move(sbus_msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(SBUSDriverNode)

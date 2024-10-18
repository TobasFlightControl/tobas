#include <tobas_constants/constants.hpp>
#include <tobas_hal_core/base_sensor_node.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/msg/sbus.hpp>

#include <tobas_aso_core/sbus.hpp>
#include <tobas_aso_core/constants.hpp>

using namespace std;

class SBUSDriverNode : public hal::BaseSensorNode
{
  using self = SBUSDriverNode;
  using super = hal::BaseSensorNode;

  static constexpr auto kInitializePeriod = 1s;

public:
  explicit SBUSDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  aso::SBUS sbus_;

  ros2::PublisherPtr<tobas_hal_msgs::msg::Sbus> sbus_pub_;
  ros2::TimerPtr initialize_timer_;

  void initializeTimerCb();
  void onPacket(const aso::SBUS::Packet& packet);
};

SBUSDriverNode::SBUSDriverNode(const rclcpp::NodeOptions& options)
  : super("aso_sbus_driver", options), sbus_(bind(&self::onPacket, this, placeholders::_1))
{
  // Advertise publisher
  sbus_pub_ = createPublisher<tobas_hal_msgs::msg::Sbus>(hal::kSbusTopic);

  // UARTの初期化は反映されないことがあるため，データが読み取れるまでタイマーで繰り返す．
  initialize_timer_ = createTimer(kInitializePeriod, &self::initializeTimerCb, this);
}

void SBUSDriverNode::initializeTimerCb()
{
  if (sbus_.packet().periods.at(0) == 0)
  {
    TOBAS_INFO("SBUS packet is not received yet. Initializing device...");
    if (!sbus_.initialize())
      TOBAS_ERROR("Failed to initialize S.BUS driver.");
  }
  else
  {
    initialize_timer_->cancel();
  }
}

void SBUSDriverNode::onPacket(const aso::SBUS::Packet& packet)
{
  // Create message
  auto sbus_msg = std::make_unique<tobas_hal_msgs::msg::Sbus>();
  sbus_msg->header.stamp = get_clock()->now();
  sbus_msg->data = packet.periods;

  // Publish message
  sbus_pub_->publish(move(sbus_msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(SBUSDriverNode)

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

public:
  explicit SBUSDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  aso::SBUS sbus_;

  ros2::PublisherPtr<tobas_hal_msgs::msg::Sbus> sbus_pub_;

  void onPacket(const aso::SBUS::Packet& packet);
};

SBUSDriverNode::SBUSDriverNode(const rclcpp::NodeOptions& options)
  : super("aso_sbus_driver", options), sbus_(bind(&self::onPacket, this, placeholders::_1))
{
  // Advertise publisher
  sbus_pub_ = createPublisher<tobas_hal_msgs::msg::Sbus>(hal::kSbusTopic);

  // 1回だと設定が反映されないことがあるため，複数回初期化リクエストを送る．
  // FIXME: 1発で確実に初期化する，もしくは初期化が成功したかどうかを確認したい．
  for (int i = 0; i < 10; ++i)
  {
    if (!sbus_.initialize())
      TOBAS_EXIT("Failed to initialize S.BUS driver.");
    get_clock()->sleep_for(10ms);
  }

  sbus_.start();
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

#include <tobas_hal_core/base_sensor_node.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/msg/sbus.hpp>

#include <tobas_aso_core/sbus.hpp>

using namespace std;

class SBUSDriverNode : public hal::BaseSensorNode
{
  static constexpr double kErrorPeriod = 1.;  // [s]

  using self = SBUSDriverNode;
  using super = hal::BaseSensorNode;

public:
  explicit SBUSDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  aso::SBUS sbus_;

  PublisherPtr<tobas_hal_msgs::msg::Sbus> sbus_pub_;

  void mainTimerCb();
};

SBUSDriverNode::SBUSDriverNode(const rclcpp::NodeOptions& options) : super("aso_sbus_driver", options)
{
  if (!sbus_.initialize())
    TOBAS_EXIT("Failed to initialize S.BUS driver.");

  sbus_pub_ = createPublisher<tobas_hal_msgs::msg::Sbus>(hal::kSbusTopic);

  // S.BUSドライバはブロッキングモードだから，メインタイマーを最大レートで回してもCPU消費は低い．
  main_timer_ = createTimer(0ns, &self::mainTimerCb, this);
}

void SBUSDriverNode::mainTimerCb()
{
  // Read S.BUS
  if (!sbus_.update())
  {
    TOBAS_ERROR_THROTTLE(kErrorPeriod, "Failed to read S.BUS.");
    return;
  }

  // Create message
  auto sbus_msg = std::make_unique<tobas_hal_msgs::msg::Sbus>();
  sbus_msg->header.stamp = get_clock()->now();
  for (size_t ch = 0; ch < sbus_msg->data.size(); ++ch)
    sbus_msg->data[ch] = sbus_.getPeriod(ch);

  // Publish message
  sbus_pub_->publish(move(sbus_msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(SBUSDriverNode)

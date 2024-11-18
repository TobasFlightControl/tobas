#include <tobas_hardware_common/base_sensor_node.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_msgs/msg/fluid_pressure_raw.hpp>

#include <tobas_aso_core/ilps22qs.hpp>

using namespace std;

class BaroDriverNode : public hardware::BaseSensorNode
{
  static constexpr auto kSamplingPeriod = 10ms;

  using self = BaroDriverNode;
  using super = hardware::BaseSensorNode;

public:
  explicit BaroDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  aso::ILPS22QS baro_;
  ros2::PublisherPtr<tobas_msgs::msg::FluidPressureRaw> baro_pub_;

  void mainTimerCb();
};

BaroDriverNode::BaroDriverNode(const rclcpp::NodeOptions& options) : super("aso_baro_driver", options)
{
  if (!baro_.initialize())
    TOBAS_EXIT("Failed to initialize Barometer.");

  baro_pub_ = createPublisher<tobas_msgs::msg::FluidPressureRaw>(real::kAirPressureTopic);
  main_timer_ = createTimer(kSamplingPeriod, &self::mainTimerCb, this);
}

void BaroDriverNode::mainTimerCb()
{
  // Create messages
  auto msg = std::make_unique<tobas_msgs::msg::FluidPressureRaw>();

  // Fill headers
  msg->header.stamp = get_clock()->now();

  // Read sensor
  if (!baro_.readPressure(msg->fluid_pressure))
  {
    TOBAS_FATAL("Failed to read barometer.");
    return;
  }

  // Publish message
  baro_pub_->publish(move(msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(BaroDriverNode)

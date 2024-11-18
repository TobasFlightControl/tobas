#include <tobas_hal_core/base_sensor_node.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs_adapter/MagneticField.hpp>

#include <tobas_aso_core/iis2mdc.hpp>

using namespace std;

class MagDriverNode : public hal::BaseSensorNode
{
  static constexpr auto kSamplingPeriod = 10ms;  // [Hz] The maximum update rate of IIS2MDC

  using self = MagDriverNode;
  using super = hal::BaseSensorNode;

public:
  explicit MagDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  aso::IIS2MDC mag_;
  ros2::PublisherPtr<tobas_hal_msgs::MagneticField> mag_pub_;

  void mainTimerCb();
};

MagDriverNode::MagDriverNode(const rclcpp::NodeOptions& options) : super("aso_mag_driver", options)
{
  if (!mag_.initialize())
    TOBAS_EXIT("Failed to initialize Magnetometer.");

  mag_pub_ = createPublisher<tobas_hal_msgs::MagneticField>(hal::kMagTopic);
  main_timer_ = createTimer(kSamplingPeriod, &self::mainTimerCb, this);
}

void MagDriverNode::mainTimerCb()
{
  // Create messages
  auto msg = std::make_unique<tobas_hal_msgs::MagneticField>();

  // Fill headers
  msg->header.stamp = get_clock()->now();

  // Read sensor
  if (!mag_.readMag(msg->magnetic_field.x(), msg->magnetic_field.y(), msg->magnetic_field.z()))
  {
    TOBAS_FATAL("Failed to read magnetometer.");
    return;
  }

  // Publish message
  mag_pub_->publish(move(msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(MagDriverNode)

#include <tobas_hardware_common/base_sensor_node.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_msgs_adapter/magnetic_field_stamped.hpp>

#include <tobas_aso_core/iis2mdc.hpp>

#include "./common.hpp"

using namespace std;

class MagDriverNode : public hardware::BaseSensorNode
{
  static constexpr auto kSamplingPeriod = 10ms;

  using self = MagDriverNode;
  using super = hardware::BaseSensorNode;

public:
  explicit MagDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  aso::IIS2MDC mag_;
  ros2::PublisherPtr<tobas_msgs::MagneticFieldStamped> mag_pub_;
  ros2::TimerPtr initialize_timer_;

  void initialize();
  void mainTimerCb();
};

MagDriverNode::MagDriverNode(const rclcpp::NodeOptions& options) : super("aso_mag_driver", options)
{
  initialize_timer_ = createTimer(aso::kRetryInitializationInterval, &self::initialize, this);
}

void MagDriverNode::initialize()
{
  if (!mag_.initialize())
  {
    TOBAS_ERROR("Failed to initialize Magnetometer. Retrying...");
    return;
  }

  mag_pub_ = createPublisher<tobas_msgs::MagneticFieldStamped>(real::kMagTopic);

  initialize_timer_.reset();
  main_timer_ = createTimer(kSamplingPeriod, &self::mainTimerCb, this);
}

void MagDriverNode::mainTimerCb()
{
  // Create messages
  auto msg = std::make_unique<tobas_msgs::MagneticFieldStamped>();

  // Fill headers
  msg->header.stamp = get_clock()->now();

  // Read sensor
  if (!mag_.readMag(msg->mag.x(), msg->mag.y(), msg->mag.z()))
  {
    TOBAS_FATAL("Failed to read magnetometer.");
    return;
  }

  // Publish message
  mag_pub_->publish(move(msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(MagDriverNode)

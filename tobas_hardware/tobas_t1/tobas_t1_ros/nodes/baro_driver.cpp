#include <tobas_hardware_common/base_sensor_node.hpp>
#include <tobas_real_common/constants.hpp>
#include <tobas_msgs/msg/fluid_pressure_stamped.hpp>

#include <tobas_t1_core/ilps22qs.hpp>

#include "./common.hpp"

using namespace std;

class BaroDriverNode : public hardware::BaseSensorNode
{
  static constexpr auto kSamplingPeriod = 10ms;

  using self = BaroDriverNode;
  using super = hardware::BaseSensorNode;

public:
  explicit BaroDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  t1::ILPS22QS baro_;
  ros2::PublisherPtr<tobas_msgs::msg::FluidPressureStamped> baro_pub_;
  ros2::TimerPtr initialize_timer_;

  void initialize();
  void mainTimerCb();
};

BaroDriverNode::BaroDriverNode(const rclcpp::NodeOptions& options) : super("t1_baro_driver", options)
{
  initialize_timer_ = createWallTimer(t1::kRetryInitializationInterval, &self::initialize, this);
}

void BaroDriverNode::initialize()
{
  if (!baro_.initialize())
  {
    TOBAS_ERROR("Failed to initialize Barometer. Retrying...");
    return;
  }

  baro_pub_ = createPublisher<tobas_msgs::msg::FluidPressureStamped>(real::kAirPressureTopic);

  initialize_timer_->cancel();
  initialize_timer_.reset();

  main_timer_ = createWallTimer(kSamplingPeriod, &self::mainTimerCb, this);
}

void BaroDriverNode::mainTimerCb()
{
  // Create messages
  auto msg = std::make_unique<tobas_msgs::msg::FluidPressureStamped>();

  // Fill headers
  msg->header.stamp = get_clock()->now();

  // Read sensor
  if (!baro_.readPressure(msg->pressure))
  {
    TOBAS_FATAL("Failed to read barometer.");
    return;
  }

  // Publish message
  baro_pub_->publish(move(msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(BaroDriverNode)

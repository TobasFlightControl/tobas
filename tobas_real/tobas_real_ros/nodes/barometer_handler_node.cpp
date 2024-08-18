#include <sensor_msgs/msg/fluid_pressure.hpp>

#include <tobas_dsp/noise_variance_filter.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/msg/fluid_pressure.hpp>

#include "../include/tobas_real_ros/common.hpp"

using namespace std;

class BarometerHandlerNode : public tobas::BaseNode
{
  static constexpr double kHpfCutoff = 10.;  // [Hz] (G(1Hz) ~ 0.1, G(20Hz) ~ 0.9)
  static constexpr size_t kWindowSize = 100;

  static constexpr double kMinAirPressure = 30000.;   // [Pa] 有効な気圧の下限 (エベレスト山頂)
  static constexpr double kMaxAirPressure = 120000.;  // [Pa] 有効な気圧の上限 (観測史上最大以上)

  using self = BarometerHandlerNode;
  using super = tobas::BaseNode;

public:
  explicit BarometerHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas_hal_msgs::msg::FluidPressure::ConstSharedPtr bar_raw_;
  dsp::NoiseVarianceFilter pressure_noise_;

  PublisherPtr<sensor_msgs::msg::FluidPressure> bar_pub_;
  SubscriberPtr<tobas_hal_msgs::msg::FluidPressure> bar_sub_;

  void airPressureCb(const tobas_hal_msgs::msg::FluidPressure::ConstSharedPtr& bar_raw);
};

BarometerHandlerNode::BarometerHandlerNode(const rclcpp::NodeOptions& options) : super("barometer_handler", options)
{
  bar_pub_ = createPublisher<sensor_msgs::msg::FluidPressure>(tobas::kAirPressureTopic);
  bar_sub_ = createSubscriber(hal::kAirPressureTopic, &self::airPressureCb, this);
}

void BarometerHandlerNode::airPressureCb(const tobas_hal_msgs::msg::FluidPressure::ConstSharedPtr& bar_raw)
{
  // Initialize
  if (bar_raw_ == nullptr)
  {
    pressure_noise_.initialize(kWindowSize, kHpfCutoff, bar_raw->fluid_pressure);
    bar_raw_ = bar_raw;
    return;
  }

  // Compute time difference
  const auto dt = (bar_raw->header.stamp - bar_raw_->header.stamp).seconds();
  bar_raw_ = bar_raw;

  // Validate
  if (bar_raw->fluid_pressure < kMinAirPressure || kMaxAirPressure < bar_raw->fluid_pressure)
  {
    TOBAS_ERROR_THROTTLE(real::kErrorPeriod, "Strange air pressure: ", bar_raw->fluid_pressure, " [Pa]");
    return;
  }

  // Update noise filter
  pressure_noise_.update(bar_raw->fluid_pressure, dt);

  // Create message
  auto bar_msg = std::make_unique<sensor_msgs::msg::FluidPressure>();
  bar_msg->header = bar_raw->header;
  bar_msg->fluid_pressure = bar_raw->fluid_pressure;
  bar_msg->variance = pressure_noise_.noiseVariance();

  // Publish message
  bar_pub_->publish(move(bar_msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(BarometerHandlerNode)

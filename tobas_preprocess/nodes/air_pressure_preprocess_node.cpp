#include <tobas_dsp/low_pass_filter.hpp>
#include <tobas_dsp/noise_variance_filter.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/fluid_pressure.hpp>
#include <tobas_msgs/msg/fluid_pressure_raw.hpp>

using namespace std;

class AirPressurePreprocessNode : public tobas::BaseNode
{
  static constexpr double kHpfCutoff = 10.;  // [Hz] (G(1Hz) ~ 0.1, G(20Hz) ~ 0.9)
  static constexpr size_t kWindowSize = 100;

  using self = AirPressurePreprocessNode;
  using super = tobas::BaseNode;

public:
  explicit AirPressurePreprocessNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas_msgs::msg::FluidPressureRaw::ConstSharedPtr pres_raw_;
  dsp::NoiseVarianceFilter pres_noise_;

  ros2::PublisherPtr<tobas_msgs::msg::FluidPressure> pres_pub_;
  ros2::SubscriberPtr<tobas_msgs::msg::FluidPressureRaw> pres_raw_sub_;

  void presRawCb(const tobas_msgs::msg::FluidPressureRaw::ConstSharedPtr& pres_raw);
};

AirPressurePreprocessNode::AirPressurePreprocessNode(const rclcpp::NodeOptions& options)
  : super("air_pressure_preprocess", options)
{
  pres_pub_ = createPublisher<tobas_msgs::msg::FluidPressure>(tobas::kAirPressureTopic);
  pres_raw_sub_ = createSubscriber(tobas::kAirPressureRawTopic, &self::presRawCb, this);
}

void AirPressurePreprocessNode::presRawCb(const tobas_msgs::msg::FluidPressureRaw::ConstSharedPtr& pres_raw)
{
  // Initialize
  if (pres_raw_ == nullptr)
  {
    pres_noise_.initialize(kWindowSize, kHpfCutoff, pres_raw->fluid_pressure);
    pres_raw_ = pres_raw;
    return;
  }

  // Compute time difference
  const auto dt = (pres_raw->header.stamp - pres_raw_->header.stamp).seconds();
  pres_raw_ = pres_raw;

  // Update noise filter
  if (pres_noise_.update(pres_raw->fluid_pressure, dt) < 0)
    TOBAS_ERROR_THROTTLE(tobas::kTypicalErrorPeriod, "Noise filter failed: ", pres_noise_.errorMessage());

  // Create message
  auto pres_out = std::make_unique<tobas_msgs::msg::FluidPressure>();
  pres_out->header = pres_raw->header;
  pres_out->fluid_pressure = pres_raw->fluid_pressure;
  pres_out->variance = pres_noise_.noiseVariance();

  // Publish message
  pres_pub_->publish(move(pres_out));
}

RCLCPP_COMPONENTS_REGISTER_NODE(AirPressurePreprocessNode)

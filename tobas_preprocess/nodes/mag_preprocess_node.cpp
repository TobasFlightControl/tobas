#include <tobas_dsp/low_pass_filter.hpp>
#include <tobas_dsp/noise_variance_filter.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs_adapter/magnetic_field_with_covariance_stamped.hpp>
#include <tobas_msgs_adapter/magnetic_field_stamped.hpp>

using namespace std;

class MagPreprocessNode : public tobas::BaseNode
{
  static constexpr double kHpfCutoff = 10.;  // [Hz] (G(1Hz) ~ 0.1, G(20Hz) ~ 0.9)
  static constexpr size_t kWindowSize = 100;

  using self = MagPreprocessNode;
  using super = tobas::BaseNode;

public:
  explicit MagPreprocessNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas_msgs::MagneticFieldStamped::ConstSharedPtr mag_raw_;
  dsp::NoiseVarianceFilter<double, 3, kWindowSize> mag_noise_;

  ros2::PublisherPtr<tobas_msgs::MagneticFieldWithCovarianceStamped> mag_pub_;
  ros2::SubscriberPtr<tobas_msgs::MagneticFieldStamped> mag_raw_sub_;

  void magRawCb(const tobas_msgs::MagneticFieldStamped::ConstSharedPtr& mag_raw);
};

MagPreprocessNode::MagPreprocessNode(const rclcpp::NodeOptions& options) : super("mag_preprocess", options)
{
  mag_pub_ = createPublisher<tobas_msgs::MagneticFieldWithCovarianceStamped>(tobas::kMagTopic);
  mag_raw_sub_ = createSubscriber(tobas::kMagRawTopic, &self::magRawCb, this);
}

void MagPreprocessNode::magRawCb(const tobas_msgs::MagneticFieldStamped::ConstSharedPtr& mag_raw)
{
  // Initialize
  if (mag_raw_ == nullptr)
  {
    mag_noise_.initialize(kHpfCutoff, mag_raw->mag.data);
    mag_raw_ = mag_raw;
    return;
  }

  // Compute time difference
  const auto dt = (mag_raw->header.stamp - mag_raw_->header.stamp).seconds();
  mag_raw_ = mag_raw;

  // Update noise filter
  if (mag_noise_.update(mag_raw->mag.data, dt) < 0)
    TOBAS_ERROR_THROTTLE(tobas::kTypicalErrorPeriod, "Noise filter failed: ", mag_noise_.errorMessage());

  // Create message
  auto mag_out = std::make_unique<tobas_msgs::MagneticFieldWithCovarianceStamped>();
  mag_out->header = mag_raw->header;
  mag_out->mag.mag = mag_raw->mag;
  mag_out->mag.covariance = mag_noise_.noiseVariance();

  // Publish message
  mag_pub_->publish(move(mag_out));
}

RCLCPP_COMPONENTS_REGISTER_NODE(MagPreprocessNode)

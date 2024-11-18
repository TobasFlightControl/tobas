#include <tobas_dsp/low_pass_filter.hpp>
#include <tobas_dsp/noise_variance_filter.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs_adapter/MagneticField.hpp>
#include <tobas_msgs_adapter/MagneticFieldRaw.hpp>

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
  tobas_msgs::MagneticFieldRaw::ConstSharedPtr mag_raw_;
  array<dsp::NoiseVarianceFilter, 3> mag_noise_;

  ros2::PublisherPtr<tobas_msgs::MagneticField> mag_pub_;
  ros2::SubscriberPtr<tobas_msgs::MagneticFieldRaw> mag_raw_sub_;

  void magRawCb(const tobas_msgs::MagneticFieldRaw::ConstSharedPtr& mag_raw);
};

MagPreprocessNode::MagPreprocessNode(const rclcpp::NodeOptions& options) : super("mag_preprocess", options)
{
  mag_pub_ = createPublisher<tobas_msgs::MagneticField>(tobas::kMagTopic);
  mag_raw_sub_ = createSubscriber(tobas::kMagRawTopic, &self::magRawCb, this);
}

void MagPreprocessNode::magRawCb(const tobas_msgs::MagneticFieldRaw::ConstSharedPtr& mag_raw)
{
  // Initialize
  if (mag_raw_ == nullptr)
  {
    for (size_t i = 0; i < 3; ++i)
      mag_noise_[i].initialize(kWindowSize, kHpfCutoff, mag_raw->magnetic_field(i));
    mag_raw_ = mag_raw;
    return;
  }

  // Compute time difference
  const auto dt = (mag_raw->header.stamp - mag_raw_->header.stamp).seconds();
  mag_raw_ = mag_raw;

  // Update noise filter
  for (size_t i = 0; i < 3; ++i)
    if (mag_noise_[i].update(mag_raw->magnetic_field(i), dt) < 0)
      TOBAS_ERROR_THROTTLE(tobas::kTypicalErrorPeriod, "Noise filter failed: ", mag_noise_[i].errorMessage());

  // Create message
  auto mag_out = std::make_unique<tobas_msgs::MagneticField>();

  // Fill header
  mag_out->header = mag_raw->header;

  // Fill data
  mag_out->magnetic_field = mag_raw->magnetic_field;

  // Fill covariance matrices
  mag_out->covariance.setZero();
  for (size_t i = 0; i < 3; ++i)
    mag_out->covariance(i, i) = mag_noise_[i].noiseVariance();

  // Publish message
  mag_pub_->publish(move(mag_out));
}

RCLCPP_COMPONENTS_REGISTER_NODE(MagPreprocessNode)

#include <tobas_algorithm/kahan.hpp>
#include <tobas_dsp/low_pass_filter.hpp>
#include <tobas_dsp/noise_variance_filter.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs_adapter/imu_stamped.hpp>
#include <tobas_msgs_adapter/imu_with_covariance_stamped.hpp>

class ImuPreprocessNode : public tobas::BaseNode
{
  static constexpr double kNoiseFiltrerHPFCutoff = 30.;  // [Hz] (G(3Hz) ~ 0.1, G(100Hz) ~ 0.95)
  static constexpr size_t kNoiseFilterWindowSize = 200;  // 400Hzで0.5s
  static constexpr int kMeasureGyroBiasCount = 1000;     // [-]
  static constexpr double kStaticGyroThreshold = 0.1;    // [rad/s]

  // Default dynamic parameters
  static constexpr long kDefaultAccelLowPassCutoff = 40;  // TODO: ノッチフィルタを導入したら上げる
  static constexpr long kDefaultGyroLowPassCutoff = 40;   // TODO: ノッチフィルタを導入したら上げる

  using self = ImuPreprocessNode;
  using super = tobas::BaseNode;

public:
  explicit ImuPreprocessNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  enum stage_t
  {
    MEASURE_GYRO_BIAS,
    INITIALIZE,
    PUBLISH,
  } stage_ = MEASURE_GYRO_BIAS;

  // ジャイロバイアス関連
  kdl::Vector gyro_bias_;
  size_t gyro_bias_cnt_ = 0;
  std::array<algo::Kahan<double>, 3> gyro_sum_;

  tobas_msgs::ImuStamped::ConstSharedPtr imu_raw_;
  dsp::LowPassFilter<kdl::Vector> acc_lpf_, gyro_lpf_;
  dsp::NoiseVarianceFilter<double, 3, kNoiseFilterWindowSize> acc_noise_, gyro_noise_;

  ros2::PublisherPtr<tobas_msgs::ImuWithCovarianceStamped> imu_pub_;
  ros2::SubscriberPtr<tobas_msgs::ImuStamped> imu_raw_sub_;

  bool accelLowPassCutoffCb(const long& p);
  bool gyroLowPassCutoffCb(const long& p);

  void imuRawCb(const tobas_msgs::ImuStamped::ConstSharedPtr& imu_raw);
};

ImuPreprocessNode::ImuPreprocessNode(const rclcpp::NodeOptions& options) : super("imu_preprocess", options)
{
  addDynamicIntParam("accel_lowpass_cutoff", &self::accelLowPassCutoffCb, this, kDefaultAccelLowPassCutoff, 30, 400);
  addDynamicIntParam("gyro_lowpass_cutoff", &self::gyroLowPassCutoffCb, this, kDefaultGyroLowPassCutoff, 30, 400);

  imu_pub_ = createPublisher<tobas_msgs::ImuWithCovarianceStamped>(tobas::kImuTopic);
  imu_raw_sub_ = createSubscriber(tobas::kImuRawTopic, &self::imuRawCb, this);
}

bool ImuPreprocessNode::accelLowPassCutoffCb(const long& p)
{
  if (!acc_lpf_.setCutoffFrequency(p))
  {
    TOBAS_ERROR("Failed to set cutoff frequency of accel low-pass filter.");
    return false;
  }

  return true;
}

bool ImuPreprocessNode::gyroLowPassCutoffCb(const long& p)
{
  if (!gyro_lpf_.setCutoffFrequency(p))
  {
    TOBAS_ERROR("Failed to set cutoff frequency of gyro low-pass filter.");
    return false;
  }

  return true;
}

void ImuPreprocessNode::imuRawCb(const tobas_msgs::ImuStamped::ConstSharedPtr& imu_raw)
{
  switch (stage_)
  {
    case MEASURE_GYRO_BIAS:
    {
      // 角速度が大きすぎる場合はやり直し
      if (imu_raw->imu.gyro.norm() > kStaticGyroThreshold)
      {
        TOBAS_WARN("Perturbation is detected while measuring gyro bias: ", imu_raw->imu.gyro, " [rad/s]. Retrying...");
        gyro_bias_cnt_ = 0;
        for (size_t i = 0; i < 3; ++i)
          gyro_sum_[i].reset();
        break;
      }

      // 角速度を加算
      for (size_t i = 0; i < 3; ++i)
        gyro_sum_[i].add(imu_raw->imu.gyro(i));

      // データが溜まったら角速度の平均をバイアスの推定値として次のステージに進む
      if (++gyro_bias_cnt_ == kMeasureGyroBiasCount)
      {
        for (size_t i = 0; i < 3; ++i)
          gyro_bias_(i) = gyro_sum_[i].get() / kMeasureGyroBiasCount;
        TOBAS_INFO("Finished measuring gyro bias. It is estimated to be: ", gyro_bias_);
        stage_ = INITIALIZE;
      }

      break;
    }
    case INITIALIZE:
    {
      acc_lpf_.setValue(imu_raw->imu.accel);
      gyro_lpf_.setValue(imu_raw->imu.gyro);

      if (!acc_noise_.initialize(kNoiseFiltrerHPFCutoff, imu_raw->imu.accel.data))
      {
        TOBAS_ERROR("Failed to initialize accel noise variance filter.");
        return;
      }
      if (!gyro_noise_.initialize(kNoiseFiltrerHPFCutoff, imu_raw->imu.gyro.data))
      {
        TOBAS_ERROR("Failed to initialize gyro noise variance filter.");
        return;
      }

      imu_raw_ = imu_raw;
      stage_ = PUBLISH;
      break;
    }
    case PUBLISH:
    {
      // Compute time difference
      const auto dt = (imu_raw->header.stamp - imu_raw_->header.stamp).seconds();
      imu_raw_ = imu_raw;

      // Low-pass filter
      acc_lpf_.update(imu_raw->imu.accel, dt);
      gyro_lpf_.update(imu_raw->imu.gyro - gyro_bias_, dt);

      const auto& acc_out = acc_lpf_.getValue();
      const auto& gyro_out = gyro_lpf_.getValue();

      // Update noise filters
      acc_noise_.update(acc_out.data, dt);
      gyro_noise_.update(gyro_out.data, dt);

      // Create IMU message
      auto imu_out = std::make_unique<tobas_msgs::ImuWithCovarianceStamped>();
      imu_out->header = imu_raw->header;
      imu_out->imu.imu.accel = acc_out;
      imu_out->imu.imu.gyro = gyro_out;
      imu_out->imu.accel_covariance = acc_noise_.noiseVariance();
      imu_out->imu.gyro_covariance = gyro_noise_.noiseVariance();

      // Publish message
      imu_pub_->publish(std::move(imu_out));

      break;
    }
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ImuPreprocessNode)

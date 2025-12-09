#include <tobas_constants/constants.hpp>
#include <tobas_dsp/noise_variance_filter.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/time.hpp>

#include <tobas_eigen_msgs_adapter/core.hpp>
#include <tobas_msgs_adapter/imu.hpp>

/**
 * @brief IMUの共分散をオンラインで計算する．
 */
class ImuNoiseFilter : public tobas::BaseNode
{
  using self = ImuNoiseFilter;
  using super = tobas::BaseNode;

  static constexpr size_t kWindowSize = 400;    // 400Hzで1秒
  static constexpr double kHpfCutoffFreq = 1.;  // [Hz]

public:
  explicit ImuNoiseFilter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  dsp::NoiseVarianceFilter<double, 3, kWindowSize> acc_noise_filter_;
  dsp::NoiseVarianceFilter<double, 3, kWindowSize> gyro_noise_filter_;

  tobas_msgs::Imu::ConstSharedPtr imu_;

  ros2::PublisherPtr<Eigen::Matrix3d> acc_noise_pub_;
  ros2::PublisherPtr<Eigen::Matrix3d> gyro_noise_pub_;
  ros2::SubscriberPtr<tobas_msgs::Imu> imu_sub_;

  void imuCb(const tobas_msgs::Imu::ConstSharedPtr& imu);
};

ImuNoiseFilter::ImuNoiseFilter(const rclcpp::NodeOptions& options) : super("imu_noise_filter", options)
{
  acc_noise_pub_ = createPublisher<Eigen::Matrix3d>("accel_covariance");
  gyro_noise_pub_ = createPublisher<Eigen::Matrix3d>("gyro_covariance");
  imu_sub_ = createSubscriber(tobas::kImuRawTopic, &self::imuCb, this);
}

void ImuNoiseFilter::imuCb(const tobas_msgs::Imu::ConstSharedPtr& imu)
{
  if (!imu_) {
    acc_noise_filter_.initialize(kHpfCutoffFreq, imu->accel.data);
    gyro_noise_filter_.initialize(kHpfCutoffFreq, imu->gyro.data);
    imu_ = imu;
    return;
  }

  const auto dt = (imu->header.stamp - imu_->header.stamp).seconds();
  imu_ = imu;

  acc_noise_filter_.update(imu->accel.data, dt);
  auto acc_cov = std::make_unique<Eigen::Matrix3d>(acc_noise_filter_.noiseVariance());
  acc_noise_pub_->publish(std::move(acc_cov));

  gyro_noise_filter_.update(imu->gyro.data, dt);
  auto gyro_cov = std::make_unique<Eigen::Matrix3d>(gyro_noise_filter_.noiseVariance());
  gyro_noise_pub_->publish(std::move(gyro_cov));
}

RCLCPP_COMPONENTS_REGISTER_NODE(ImuNoiseFilter)

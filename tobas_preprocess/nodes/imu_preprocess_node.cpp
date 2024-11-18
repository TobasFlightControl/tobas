#include <tobas_algorithm/kahan.hpp>
#include <tobas_dsp/low_pass_filter.hpp>
#include <tobas_dsp/noise_variance_filter.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs_adapter/imu_with_covariance_stamped.hpp>
#include <tobas_msgs_adapter/imu_stamped.hpp>

using namespace std;

class ImuPreprocessNode : public tobas::BaseNode
{
  // マルチコプターのプロペラの回転数は 3000~12000 [RPM]．
  // ブレード数が n = 2 だとすると，ノイズの基本周波数は RPM / 60 * n = 100~600 [Hz] になる．
  // そこで，カットオフ周波数を，3Hzの主成分を遅延4msで99%残しつつ200Hzのノイズを20%に減衰させる40Hzに設定する．
  // PX4, ArduPilotはカットオフを極力大きくすべきだと言うが，誤った値がEKFに入るのを防ぐほうが重要ではないか．．．？
  // TODO: モータの回転数を取得し，ノッチフィルタで局所的に除去する (Dynamic Notch Filter)
  static constexpr double kGyroLpfCutoff = 40.;
  static constexpr double kAccelLpfCutoff = 40.;

  static constexpr double kHpfCutoff = 30.;            // [Hz] (G(3Hz) ~ 0.1, G(100Hz) ~ 0.95)
  static constexpr size_t kWindowSize = 200;           // 400Hzで0.5s
  static constexpr int kMeasureGyroBiasCount = 1000;   // [-]
  static constexpr double kStaticGyroThreshold = 0.5;  // [rad/s]

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
  array<algo::Kahan<double>, 3> gyro_sum_;

  tobas_msgs::ImuStamped::ConstSharedPtr imu_raw_;
  dsp::LowPassFilter<kdl::Vector> acc_lpf_, gyro_lpf_;
  array<dsp::NoiseVarianceFilter, 3> acc_noise_, gyro_noise_;

  ros2::PublisherPtr<tobas_msgs::ImuWithCovarianceStamped> imu_pub_;
  ros2::SubscriberPtr<tobas_msgs::ImuStamped> imu_raw_sub_;

  void imuRawCb(const tobas_msgs::ImuStamped::ConstSharedPtr& imu_raw);
};

ImuPreprocessNode::ImuPreprocessNode(const rclcpp::NodeOptions& options) : super("imu_preprocess", options)
{
  imu_pub_ = createPublisher<tobas_msgs::ImuWithCovarianceStamped>(tobas::kImuTopic);
  imu_raw_sub_ = createSubscriber(tobas::kImuRawTopic, &self::imuRawCb, this);
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
      acc_lpf_.initialize(kAccelLpfCutoff, imu_raw->imu.accel);
      gyro_lpf_.initialize(kGyroLpfCutoff, imu_raw->imu.gyro);
      for (size_t i = 0; i < 3; ++i)
      {
        acc_noise_[i].initialize(kWindowSize, kHpfCutoff, imu_raw->imu.accel(i));
        gyro_noise_[i].initialize(kWindowSize, kHpfCutoff, imu_raw->imu.gyro(i));
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

      // Filter out high frequency noise
      if (acc_lpf_.update(imu_raw->imu.accel, dt) < 0)
        TOBAS_ERROR("Failed to update accel LPF: ", acc_lpf_.errorMessage(), " dt = ", dt);
      if (gyro_lpf_.update(imu_raw->imu.gyro - gyro_bias_, dt) < 0)
        TOBAS_ERROR("Failed to update gyro LPF: ", gyro_lpf_.errorMessage(), " dt = ", dt);

      // Get filtered data
      const auto& acc = acc_lpf_.getOutput();
      const auto& gyro = gyro_lpf_.getOutput();

      // Update noise filters
      for (size_t i = 0; i < 3; ++i)
      {
        if (acc_noise_[i].update(acc(i), dt) < 0)
          TOBAS_ERROR_THROTTLE(tobas::kTypicalErrorPeriod, "Accel noise filter failed: ", acc_noise_[i].errorMessage());
        if (gyro_noise_[i].update(gyro(i), dt) < 0)
          TOBAS_ERROR_THROTTLE(tobas::kTypicalErrorPeriod, "Gyro noise filter failed: ", gyro_noise_[i].errorMessage());
      }

      // Create IMU message
      auto imu_out = std::make_unique<tobas_msgs::ImuWithCovarianceStamped>();

      // Fill header
      imu_out->header = imu_raw->header;

      // Fill data
      imu_out->imu.imu.accel = acc;
      imu_out->imu.imu.gyro = gyro;

      // Fill covariance matrices
      imu_out->imu.accel_covariance.setZero();
      imu_out->imu.gyro_covariance.setZero();
      for (size_t i = 0; i < 3; ++i)
      {
        imu_out->imu.accel_covariance(i, i) = acc_noise_[i].noiseVariance();
        imu_out->imu.gyro_covariance(i, i) = gyro_noise_[i].noiseVariance();
      }

      // Publish message
      imu_pub_->publish(move(imu_out));

      break;
    }
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ImuPreprocessNode)

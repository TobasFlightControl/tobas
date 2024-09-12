#include <std_srvs/srv/trigger.hpp>

#include <tobas_algorithm/kahan.hpp>
#include <tobas_linux/core.hpp>
#include <tobas_property_tree/property_tree.hpp>
#include <tobas_dsp/noise_variance_filter.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs_adapter/Imu.hpp>
#include <tobas_msgs_adapter/Imu.hpp>

#include <tobas_real_common/constants.hpp>

using namespace std;
using namespace real::handler::imu;

class ImuHandlerNode : public tobas::BaseNode
{
  static constexpr double kHpfCutoff = 30.;            // [Hz] (G(3Hz) ~ 0.1, G(100Hz) ~ 0.95)
  static constexpr size_t kWindowSize = 200;           // 400Hzで0.5s
  static constexpr int kMeasureGyroBiasCount = 1000;   // [-]
  static constexpr double kStaticGyroThreshold = 0.5;  // [rad/s]

  using self = ImuHandlerNode;
  using super = tobas::BaseNode;

public:
  explicit ImuHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  enum stage_t
  {
    MEASURE_GYRO_BIAS,
    INITIALIZE,
    PUBLISH,
  } stage_ = MEASURE_GYRO_BIAS;

  // Config
  kdl::Vector acc_bias_;  // [m/s^2]

  // ジャイロバイアス関連
  kdl::Vector gyro_bias_;
  size_t gyro_bias_cnt_ = 0;
  std::array<algo::Kahan<double>, 3> gyro_sum_;

  tobas_hal_msgs::Imu::ConstSharedPtr imu_raw_;
  ptree::PropertyTree pt_;
  std::array<dsp::NoiseVarianceFilter, 3> acc_noise_, gyro_noise_;

  ros2::PublisherPtr<tobas_msgs::Imu> imu_pub_;
  ros2::SubscriberPtr<tobas_hal_msgs::Imu> imu_sub_;

  void readConfig();

  bool paramsCb(const std::vector<double>& params);
  void imuCb(const tobas_hal_msgs::Imu::ConstSharedPtr& imu_raw);
};

ImuHandlerNode::ImuHandlerNode(const rclcpp::NodeOptions& options) : super("imu_handler", options)
{
  if (!pt_.initialize(linux::expandUser(kIniPath)))
    TOBAS_EXIT("Failed to initialize property tree.");

  readConfig();

  addDynamicDoubleArrayParam(real::handler::kParamName, &self::paramsCb, this);

  imu_pub_ = createPublisher<tobas_msgs::Imu>(tobas::kImuTopic);
  imu_sub_ = createSubscriber(hal::kImuTopic, &self::imuCb, this);
}

void ImuHandlerNode::readConfig()
{
  if (!pt_.get(kOffsetXKey, acc_bias_.x()))
  {
    TOBAS_WARN("Failed to get \"", kOffsetXKey, "\". from configuration file. Accel bias is set to zero.");
    acc_bias_.setZero();
    return;
  }

  if (!pt_.get(kOffsetYKey, acc_bias_.y()))
  {
    TOBAS_WARN("Failed to get \"", kOffsetXKey, "\". from configuration file. Accel bias is set to zero.");
    acc_bias_.setZero();
    return;
  }

  if (!pt_.get(kOffsetZKey, acc_bias_.z()))
  {
    TOBAS_WARN("Failed to get \"", kOffsetXKey, "\". from configuration file. Accel bias is set to zero.");
    acc_bias_.setZero();
    return;
  }
}

bool ImuHandlerNode::paramsCb(const std::vector<double>& params)
{
  // Skip first call
  if (params.size() == 0)
    return false;

  // Check size
  if (params.size() != kParamSize)
  {
    TOBAS_ERROR("Parameter size mismatch.");
    return false;
  }

  // Update parameters
  acc_bias_.x(params.at(kOffsetXChannel));
  acc_bias_.y(params.at(kOffsetYChannel));
  acc_bias_.z(params.at(kOffsetZChannel));

  // Save parameters
  pt_.set(kOffsetXKey, params.at(kOffsetXChannel));
  pt_.set(kOffsetYKey, params.at(kOffsetYChannel));
  pt_.set(kOffsetZKey, params.at(kOffsetZChannel));
  if (pt_.save())
  {
    TOBAS_ERROR("Failed to save parameters.");
    return false;
  }

  return true;
}

void ImuHandlerNode::imuCb(const tobas_hal_msgs::Imu::ConstSharedPtr& imu_raw)
{
  switch (stage_)
  {
    case MEASURE_GYRO_BIAS:
    {
      // 角速度が大きすぎる場合はやり直し
      if (imu_raw->gyro.norm() > kStaticGyroThreshold)
      {
        TOBAS_WARN("Perturbation is detected while measuring gyro bias: ", imu_raw->gyro, " [rad/s]. Retrying...");
        gyro_bias_cnt_ = 0;
        for (size_t i = 0; i < 3; ++i)
          gyro_sum_[i].reset();
        break;
      }

      // 角速度を加算
      for (size_t i = 0; i < 3; ++i)
        gyro_sum_[i].add(imu_raw->gyro(i));

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
      for (size_t i = 0; i < 3; ++i)
      {
        acc_noise_[i].initialize(kWindowSize, kHpfCutoff, imu_raw->accel(i));
        gyro_noise_[i].initialize(kWindowSize, kHpfCutoff, imu_raw->gyro(i));
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

      // Update noise filters
      for (size_t i = 0; i < 3; ++i)
      {
        if (acc_noise_[i].update(imu_raw->accel(i), dt) < 0)
          TOBAS_ERROR_THROTTLE(tobas::kTypicalErrorPeriod, "Accel noise filter failed: ", acc_noise_[i].errorMessage());
        if (gyro_noise_[i].update(imu_raw->gyro(i), dt) < 0)
          TOBAS_ERROR_THROTTLE(tobas::kTypicalErrorPeriod, "Gyro noise filter failed: ", gyro_noise_[i].errorMessage());
      }

      // Create message
      auto imu_msg = std::make_unique<tobas_msgs::Imu>();

      // Fill header
      imu_msg->header = imu_raw->header;

      // Fill data
      imu_msg->accel = imu_raw->accel - acc_bias_;
      imu_msg->gyro = imu_raw->gyro - gyro_bias_;

      // Fill covariance matrices
      imu_msg->accel_covariance.setZero();
      imu_msg->gyro_covariance.setZero();
      for (size_t i = 0; i < 3; ++i)
      {
        imu_msg->accel_covariance(i, i) = acc_noise_[i].noiseVariance();
        imu_msg->gyro_covariance(i, i) = gyro_noise_[i].noiseVariance();
      }

      // Publish message
      imu_pub_->publish(move(imu_msg));

      break;
    }
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ImuHandlerNode)

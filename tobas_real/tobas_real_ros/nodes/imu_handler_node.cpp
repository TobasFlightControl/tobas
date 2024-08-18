#include <array>
#include <std_srvs/srv/trigger.hpp>

#include <tobas_algorithm/kahan.hpp>
#include <tobas_dsp/noise_variance_filter.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_property_tools/property_client.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/Imu.hpp>
#include <tobas_msgs/Imu.hpp>

#include <tobas_real_common/constants.hpp>

using namespace std;

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
  ptree::PropertyClient::SharedPtr property_client_;
  std::array<dsp::NoiseVarianceFilter, 3> acc_noise_, gyro_noise_;

  PublisherPtr<tobas_msgs::Imu> imu_pub_;
  SubscriberPtr<tobas_hal_msgs::Imu> imu_sub_;
  ServicePtr<std_srvs::srv::Trigger> reload_config_srv_;

  TimerPtr initialize_timer_;
  void initializeTimerCb();

  bool reloadConfig();

  void imuCb(const tobas_hal_msgs::Imu::ConstSharedPtr& imu_raw);
  void reloadConfigCb(
    const std_srvs::srv::Trigger::Request::ConstSharedPtr& req,
    const std_srvs::srv::Trigger::Response::SharedPtr& res);
};

ImuHandlerNode::ImuHandlerNode(const rclcpp::NodeOptions& options) : super("imu_handler", options)
{
  initialize_timer_ = createTimer(0ns, &self::initializeTimerCb, this);
}

void ImuHandlerNode::initializeTimerCb()
{
  property_client_ = std::make_shared<ptree::PropertyClient>(shared_from_this(), real::kPropertyServerFC);
  reloadConfig();

  imu_pub_ = createPublisher<tobas_msgs::Imu>(tobas::kImuTopic);
  imu_sub_ = createSubscriber(hal::kImuTopic, &self::imuCb, this);

  reload_config_srv_ =
    createService<std_srvs::srv::Trigger>(name() + tobas::kReloadConfigSrvSuffix, &self::reloadConfigCb, this);

  initialize_timer_->cancel();
}

bool ImuHandlerNode::reloadConfig()
{
  if (property_client_->get(real::kConfigKey_AccOffsetX, acc_bias_.x()) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    acc_bias_.setZero();
    return false;
  }
  if (property_client_->get(real::kConfigKey_AccOffsetY, acc_bias_.y()) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    acc_bias_.setZero();
    return false;
  }
  if (property_client_->get(real::kConfigKey_AccOffsetZ, acc_bias_.z()) < 0)
  {
    TOBAS_ERROR(property_client_->errorMessage());
    acc_bias_.setZero();
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
        acc_noise_[i].update(imu_raw->accel(i), dt);
        gyro_noise_[i].update(imu_raw->gyro(i), dt);
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

void ImuHandlerNode::reloadConfigCb(
  const std_srvs::srv::Trigger::Request::ConstSharedPtr&,
  const std_srvs::srv::Trigger::Response::SharedPtr& res)
{
  if (!reloadConfig())
  {
    res->success = false;
    res->message = "Failed to reload configurations.";
    return;
  }

  res->success = true;
  res->message.clear();
}

RCLCPP_COMPONENTS_REGISTER_NODE(ImuHandlerNode)

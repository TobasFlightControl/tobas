#pragma once

#include <array>
#include <std_srvs/srv/trigger.hpp>

#include <tobas_algorithm/kahan.hpp>
#include <tobas_property_tools/property_client.hpp>
#include <tobas_dsp/noise_variance_filter.hpp>
#include <tobas_node/node.hpp>
#include <tobas_hal_msgs/Imu.h>

namespace tobas_real_ros
{
class ImuHandler : public tobas::BaseNode
{
  static constexpr double kHpfCutoff = 30.;            // [Hz] (G(3Hz) ~ 0.1, G(100Hz) ~ 0.95)
  static constexpr size_t kWindowSize = 200;           // 400Hzで0.5s
  static constexpr int kMeasureGyroBiasCount = 1000;   // [-]
  static constexpr double kStaticGyroThreshold = 0.5;  // [rad/s]

  using self = ImuHandler;
  using super = tobas::BaseNode;

public:
  explicit ImuHandler(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

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
  ptree::PropertyClient property_client_;
  std::array<dsp::NoiseVarianceFilter, 3> acc_noise_, gyro_noise_;

  PublisherPtr<> imu_pub_;
  SubscriberPtr<> imu_sub_;
  ServicePtr<> reload_config_srv_;

  bool reloadConfig();

  void imuCb(const tobas_hal_msgs::Imu::ConstSharedPtr& imu_raw);
  bool reloadConfigCb(std_srvs::srv::Trigger::Request& req, std_srvs::srv::Trigger::Response& res);
};
}  // namespace tobas_real_ros

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_dsp/noise_variance_filter.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/time.hpp>

#include <tobas_eigen_msgs_adapter/core.hpp>
#include <tobas_msgs_adapter/imu.hpp>

namespace tobas
{
/** Calculate IMU covariance online. */
class ImuNoiseFilter : public BaseNode
{
  using self = ImuNoiseFilter;
  using super = BaseNode;

public:
  explicit ImuNoiseFilter(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());

private:
  static constexpr size_t kWindowSize = 400;  ///< 1 second at 400 Hz
  dsp::NoiseVarianceFilter<double, 3, kWindowSize> acc_noise_filter_;
  dsp::NoiseVarianceFilter<double, 3, kWindowSize> gyro_noise_filter_;

  tobas_msgs::Imu::ConstSharedPtr imu_;

  ros2::PublisherPtr<Eigen::Matrix3d> acc_noise_pub_;
  ros2::PublisherPtr<Eigen::Matrix3d> gyro_noise_pub_;
  ros2::SubscriberPtr<tobas_msgs::Imu> imu_sub_;

  void imuCb(const tobas_msgs::Imu::ConstSharedPtr& _imu);
};

ImuNoiseFilter::ImuNoiseFilter(const rclcpp::NodeOptions& _options)
  : super("imu_noise_filter", nodeOptions_Default(_options))
{
  acc_noise_pub_ = createPublisher<Eigen::Matrix3d>("accel_covariance");
  gyro_noise_pub_ = createPublisher<Eigen::Matrix3d>("gyro_covariance");
  imu_sub_ = createSubscriber(topic::kImuRaw, &self::imuCb, this);
}

void ImuNoiseFilter::imuCb(const tobas_msgs::Imu::ConstSharedPtr& _imu)
{
  if (!imu_) {
    constexpr double kHpfCutoffFreq = 1.0;  // [Hz]
    acc_noise_filter_.initialize(kHpfCutoffFreq, _imu->accel.data);
    gyro_noise_filter_.initialize(kHpfCutoffFreq, _imu->gyro.data);
    imu_ = _imu;
    return;
  }

  const auto dt = (_imu->header.stamp - imu_->header.stamp).seconds();
  imu_ = _imu;

  acc_noise_filter_.update(_imu->accel.data, dt);
  auto acc_cov = std::make_unique<Eigen::Matrix3d>(acc_noise_filter_.noiseVariance());
  acc_noise_pub_->publish(std::move(acc_cov));

  gyro_noise_filter_.update(_imu->gyro.data, dt);
  auto gyro_cov = std::make_unique<Eigen::Matrix3d>(gyro_noise_filter_.noiseVariance());
  gyro_noise_pub_->publish(std::move(gyro_cov));
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::ImuNoiseFilter)

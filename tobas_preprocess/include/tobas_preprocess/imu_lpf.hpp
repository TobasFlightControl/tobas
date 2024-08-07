#pragma once

#include <tobas_dsp/low_pass_filter.hpp>
#include <tobas_node/node.hpp>
#include <tobas_msgs/Imu.h>

namespace tobas_preprocess
{
class ImuLpf : public tobas::BaseNode
{
  static constexpr double kGyroLpfCutoff = 40;   // [Hz] Same as the default IMU_GYRO_CUTOFF (PX4)
  static constexpr double kAccelLpfCutoff = 30;  // [Hz] Same as the default IMU_ACCEL_CUTOFF (PX4)

  using self = ImuLpf;
  using super = tobas::BaseNode;

public:
  explicit ImuLpf(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  dsp::LowPassFilter<kdl::Vector> gyro_lpf_, accel_lpf_;
  tobas_msgs::Imu::ConstSharedPtr last_msg_;

  PublisherPtr<> imu_lpf_pub_;
  SubscriberPtr<> imu_raw_sub_;

  void imuRawCb(const tobas_msgs::Imu::ConstSharedPtr& imu_raw);
};
}  // namespace tobas_preprocess

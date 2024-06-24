#pragma once

#include <tobas_dsp/low_pass_filter.hpp>
#include <tobas_tools/node.hpp>
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
  explicit ImuLpf(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name = ros::this_node::getName());

private:
  dsp::LowPassFilter<kdl::Vector> gyro_lpf_, accel_lpf_;
  tobas_msgs::ImuConstPtr last_msg_;

  ros::Publisher imu_lpf_pub_;
  ros::Subscriber imu_raw_sub_;

  void imuRawCb(const tobas_msgs::ImuConstPtr& imu_raw);
};
}  // namespace tobas_preprocess

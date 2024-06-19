#pragma once

#include <tobas_std_tools/first_order_filter.hpp>
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
  explicit ImuLpf(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  tobas_std::FirstOrderFilter<tobas_kdl::Vector> gyro_lpf_, accel_lpf_;
  ros::Time t_last_;

  ros::Publisher imu_lpf_pub_;
  ros::Subscriber imu_raw_sub_;

  void imuRawCb(const tobas_msgs::ImuConstPtr& imu_raw);
};
}  // namespace tobas_preprocess

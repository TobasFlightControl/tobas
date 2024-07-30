#pragma once

#include <eigen3/Eigen/Core>

#include <tobas_hal_core/base_sensor_node.hpp>
#include <tobas_navio_core/lsm9ds1.hpp>

namespace tobas_navio_ros
{
class ImuHandler : public hal::BaseSensorNode
{
  static constexpr size_t kSamplingRate = 400;  // [Hz]

  using self = ImuHandler;
  using super = hal::BaseSensorNode;

public:
  explicit ImuHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name = ros::this_node::getName());

private:
  navio::LSM9DS1 imu_;
  Eigen::Vector3f acc_, gyro_;
  ros::Publisher imu_pub_;

  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_navio_ros

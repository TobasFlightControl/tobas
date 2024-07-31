#pragma once

#include <eigen3/Eigen/Core>

#include <tobas_hal_core/base_sensor_node.hpp>
#include <tobas_navio_core/lsm9ds1.hpp>

namespace tobas_navio_ros
{
class MagnetometerHandler : public hal::BaseSensorNode
{
  static constexpr size_t kSamplingRate = 80;  // [Hz] LSM9DS1の地磁気のサンプリングレートの最大値

  using self = MagnetometerHandler;
  using super = hal::BaseSensorNode;

public:
  explicit MagnetometerHandler(
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

private:
  navio::LSM9DS1 imu_;
  Eigen::Vector3f mag_;
  rclcpp::Publisher mag_pub_;

  void mainTimerCb(const rclcpp::TimerEvent& event);
};
}  // namespace tobas_navio_ros

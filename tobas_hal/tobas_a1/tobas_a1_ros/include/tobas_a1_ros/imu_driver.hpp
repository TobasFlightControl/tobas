#pragma once

#include <tobas_hal_core/base_sensor_node.hpp>
#include <tobas_a1_core/ism330dlc.hpp>

namespace a1
{
class IMUDriver : public hal::BaseSensorNode
{
  static constexpr size_t kSamplingRate = 800;  // [Hz]

  using self = IMUDriver;
  using super = hal::BaseSensorNode;

public:
  explicit IMUDriver(, const std::string& name = rclcpp::this_node::getName());

private:
  ISM330DLC imu_;
  rclcpp::Publisher imu_pub_;

  void mainTimerCb(const rclcpp::TimerEvent& event);
};
}  // namespace a1

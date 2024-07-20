#pragma once

#include <tobas_kdl/vector.hpp>

#include <tobas_hal_core/base_sensor_node.hpp>
#include <tobas_a1_core/ism330dlc.hpp>

namespace a1
{
class IMUHandler : public hal::BaseSensorNode
{
  static constexpr size_t kSamplingRate = 800;  // [Hz]

  using self = IMUHandler;
  using super = hal::BaseSensorNode;

public:
  explicit IMUHandler(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name = ros::this_node::getName());

private:
  ISM330DLC imu_;
  ros::Publisher imu_pub_;

  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace a1

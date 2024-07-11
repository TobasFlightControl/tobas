#pragma once

#include <tobas_hal_core/base_sensor_node.hpp>
#include <tobas_navio_core/rc_input.hpp>

namespace tobas_navio_ros
{
class RCInputHandler : public hal::BaseSensorNode
{
  static constexpr size_t kSamplingRate = 100;  // [Hz]

  using self = RCInputHandler;
  using super = hal::BaseSensorNode;

public:
  explicit RCInputHandler(
    ros::NodeHandle& nh,
    ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  navio::RCInput rcin_;
  ros::Publisher rcin_pub_;

  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_navio_ros

#pragma once

#include <tobas_hal_core/base_sensor_node.hpp>
#include <tobas_navio_core/ms5611.hpp>

namespace tobas_navio_ros
{
class BarometerHandler : public hal::BaseSensorNode
{
  static constexpr size_t kSamplingRate = 50;  // [Hz]

  using self = BarometerHandler;
  using super = hal::BaseSensorNode;

public:
  explicit BarometerHandler(
    ros::NodeHandle& nh,
    ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  navio::MS5611 barometer_;
  ros::Publisher bar_pub_;

  void initialize();

  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace tobas_navio_ros

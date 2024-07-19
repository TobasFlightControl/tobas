#pragma once

#include <tobas_hal_core/base_sensor_node.hpp>
#include <tobas_a1_core/sbus.hpp>

namespace a1
{
class SBUSDriver : public hal::BaseSensorNode
{
  using self = SBUSDriver;
  using super = hal::BaseSensorNode;

public:
  explicit SBUSDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name = ros::this_node::getName());

private:
  SBUS sbus_;

  ros::Publisher sbus_pub_;

  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace a1

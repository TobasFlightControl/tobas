#pragma once

#include <tobas_hal_core/base_sensor_node.hpp>
#include <tobas_a1_core/iis2mdc.hpp>

namespace a1
{
class MagDriver : public hal::BaseSensorNode
{
  static constexpr size_t kSamplingRate = 100;  // [Hz] The maximum update rate of IIS2MDC

  using self = MagDriver;
  using super = hal::BaseSensorNode;

public:
  explicit MagDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name = ros::this_node::getName());

private:
  IIS2MDC mag_;
  ros::Publisher mag_pub_;

  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace a1

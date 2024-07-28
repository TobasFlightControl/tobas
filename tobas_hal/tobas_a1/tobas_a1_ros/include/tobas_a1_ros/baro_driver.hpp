#pragma once

#include <tobas_hal_core/base_sensor_node.hpp>
#include <tobas_a1_core/ilps22qs.hpp>

namespace a1
{
class BaroDriver : public hal::BaseSensorNode
{
  static constexpr size_t kSamplingRate = 100;

  using self = BaroDriver;
  using super = hal::BaseSensorNode;

public:
  explicit BaroDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name = ros::this_node::getName());

private:
  ILPS22QS baro_;
  ros::Publisher baro_pub_;

  void mainTimerCb(const ros::TimerEvent& event);
};
}  // namespace a1

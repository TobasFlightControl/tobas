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
  explicit BaroDriver(rclcpp::Node::SharedPtr node, rclcpp::Node::SharedPtr pnh, const std::string& name = rclcpp::this_node::getName());

private:
  ILPS22QS baro_;
  rclcpp::Publisher baro_pub_;

  void mainTimerCb(const rclcpp::TimerEvent& event);
};
}  // namespace a1

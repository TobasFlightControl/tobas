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
  explicit SBUSDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  SBUS sbus_;

  PublisherPtr<> sbus_pub_;

  void mainTimerCb(const rclcpp::TimerEvent& event);
};
}  // namespace a1

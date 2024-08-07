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
  explicit BaroDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ILPS22QS baro_;
  PublisherPtr<> baro_pub_;

  void mainTimerCb(const rclcpp::TimerEvent& event);
};
}  // namespace a1

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
  explicit MagDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  IIS2MDC mag_;
  PublisherPtr<> mag_pub_;

  void mainTimerCb();
};
}  // namespace a1

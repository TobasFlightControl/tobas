// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_constants/time.hpp>
#include <tobas_fc1xx_core/battery.hpp>
#include <tobas_hardware_common/base_sensor_node.hpp>

#include <tobas_msgs/msg/battery.hpp>

#include "./common.hpp"

using namespace std::chrono_literals;

namespace tobas
{
namespace fc1xx
{
class BatteryDriverNode : public hardware::BaseSensorNode
{
  static constexpr auto kSamplingPeriod = 100ms;  // TODO: Increase this after properly separating SPI devices.

  using self = BatteryDriverNode;
  using super = hardware::BaseSensorNode;

public:
  explicit BatteryDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  Battery battery_;
  float voltage_, current_;

  ros2::PublisherPtr<tobas_msgs::msg::Battery> battery_pub_;
  ros2::TimerPtr initialize_timer_;

  void initialize();
  void mainTimerCb();
};

BatteryDriverNode::BatteryDriverNode(const rclcpp::NodeOptions& options)
  : super("fc1xx_battery_driver", nodeOptions_Default(options))
{
  initialize_timer_ = createWallTimer(kRetryInitializationInterval, &self::initialize, this);
}

void BatteryDriverNode::initialize()
{
  if (!battery_.initialize()) {
    TOBAS_ERROR("Failed to initialize battery driver. Retrying...");
    return;
  }

  battery_pub_ = createPublisher<tobas_msgs::msg::Battery>(topic::kBattery);

  initialize_timer_->cancel();
  main_timer_ = createWallTimer(kSamplingPeriod, &self::mainTimerCb, this);
}

void BatteryDriverNode::mainTimerCb()
{
  // Read data
  if (!battery_.read(voltage_, current_)) {
    TOBAS_ERROR("Failed to read battery state.");
    return;
  }

  // Check data
  if (voltage_ <= 0.0) {
    TOBAS_WARN_THROTTLE(kTypicalWarnPeriod, "Battery state is unavailable.");
    return;
  }

  // Publish message
  auto msg = std::make_unique<tobas_msgs::msg::Battery>();
  msg->header.stamp = now();
  msg->voltage = static_cast<double>(voltage_);
  msg->current = static_cast<double>(current_);
  battery_pub_->publish(std::move(msg));
}
}  // namespace fc1xx
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::fc1xx::BatteryDriverNode)

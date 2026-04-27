// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_ic_drivers/stmicro/ilps22qs.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs/msg/fluid_pressure.hpp>

#include "./common.hpp"

using namespace std::chrono_literals;

namespace tobas
{
namespace fc2xx
{
class BaroDriverNode : public BaseNode
{
  static constexpr auto kSamplingPeriod = 20ms;

  using self = BaroDriverNode;
  using super = BaseNode;

public:
  explicit BaroDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  stm::ILPS22QS baro_;
  ros2::PublisherPtr<tobas_msgs::msg::FluidPressure> baro_pub_;
  ros2::TimerPtr initialize_timer_, main_timer_;

  void initialize();
  void mainTimerCb();
};

BaroDriverNode::BaroDriverNode(const rclcpp::NodeOptions& options)
  : super("fc2xx_baro_driver", nodeOptions_Default(options))
{
  initialize_timer_ = createWallTimer(kRetryInitializationInterval, &self::initialize, this);
}

void BaroDriverNode::initialize()
{
  if (!baro_.initialize("/dev/i2c-1")) {
    TOBAS_ERROR("Failed to initialize Barometer. Retrying...");
    return;
  }

  baro_pub_ = createPublisher<tobas_msgs::msg::FluidPressure>(topic::kAirPressure);

  initialize_timer_->cancel();
  main_timer_ = createWallTimer(kSamplingPeriod, &self::mainTimerCb, this);
}

void BaroDriverNode::mainTimerCb()
{
  // Create messages
  auto msg = std::make_unique<tobas_msgs::msg::FluidPressure>();

  // Fill headers
  msg->header.stamp = now();

  // Read sensor
  if (!baro_.readPressure(msg->pressure)) {
    TOBAS_FATAL("Failed to read barometer.");
    return;
  }

  // Publish message
  baro_pub_->publish(std::move(msg));
}
}  // namespace fc2xx
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::fc2xx::BaroDriverNode)

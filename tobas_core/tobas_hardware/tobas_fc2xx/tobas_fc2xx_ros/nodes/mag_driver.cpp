// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_ic_drivers/stmicro/iis2mdc.hpp>
#include <tobas_node/node.hpp>
#include <tobas_real_common/ros_interface.hpp>

#include <tobas_msgs_adapter/magnetic_field.hpp>

#include "./common.hpp"

using namespace std::chrono_literals;

namespace tobas
{
namespace fc2xx
{
class MagDriverNode : public BaseNode
{
  static constexpr auto kSamplingPeriod = 10ms;

  using self = MagDriverNode;
  using super = BaseNode;

public:
  explicit MagDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  stm::IIS2MDC mag_;
  double mx_, my_, mz_;

  ros2::PublisherPtr<tobas_msgs::MagneticField> mag_pub_;
  ros2::TimerPtr initialize_timer_, main_timer_;

  bool initialize();
  void registerRosInterfaces();

  void initializeTimerCb();
  void mainTimerCb();
};

MagDriverNode::MagDriverNode(const rclcpp::NodeOptions& options)
  : super("fc2xx_mag_driver", nodeOptions_Default(options))
{
  if (initialize()) {
    registerRosInterfaces();
  }
  else {
    initialize_timer_ = createWallTimer(kRetryInitializationInterval, &self::initializeTimerCb, this);
  }
}

bool MagDriverNode::initialize()
{
  if (!mag_.initialize("/dev/i2c-1")) {
    TOBAS_ERROR("Failed to initialize Magnetometer. Retrying...");
    return false;
  }

  return true;
}

void MagDriverNode::registerRosInterfaces()
{
  mag_pub_ = createPublisher<tobas_msgs::MagneticField>(real::topic::kMagneticField);

  main_timer_ = createWallTimer(kSamplingPeriod, &self::mainTimerCb, this);
}

void MagDriverNode::initializeTimerCb()
{
  if (initialize()) {
    registerRosInterfaces();
    initialize_timer_->cancel();
  }
}

void MagDriverNode::mainTimerCb()
{
  // Read sensor.
  if (!mag_.readMag(mx_, my_, mz_)) {
    TOBAS_FATAL("Failed to read magnetometer.");
    return;
  }

  // Create a message.
  auto msg = std::make_unique<tobas_msgs::MagneticField>();
  msg->header.stamp = now();
  msg->mag.x(my_);
  msg->mag.y(-mx_);
  msg->mag.z(-mz_);

  // Publish the message.
  mag_pub_->publish(std::move(msg));
}
}  // namespace fc2xx
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::fc2xx::MagDriverNode)

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <chrono>
#include <memory>
#include <utility>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_node/node.hpp>

#include <tobas_drone_msgs_adapter/drone.hpp>

using namespace std::chrono_literals;

namespace tobas
{
class DroneServerNode : public BaseNode
{
  using self = DroneServerNode;
  using super = BaseNode;

public:
  explicit DroneServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  Drone drone_;
  ros2::PublisherPtr<Drone> drone_pub_;
  ros2::TimerPtr initial_publish_timer_;

  void initialPublishTimerCb();
};

DroneServerNode::DroneServerNode(const rclcpp::NodeOptions& options)
  : super("drone_server", nodeOptions_Default(options))
{
  // Load a drone configuration.
  const auto tbsdrn_path = getStringParam("tbsdrn_path");
  if (const auto result = drone_.load(tbsdrn_path); !result) {
    TOBAS_ERROR("Failed to load drone configuration from '", tbsdrn_path, "': ", result.error());
    return;
  }

  // Validate the configuration.
  if (const auto result = drone_.validate(); !result) {
    TOBAS_ERROR("Drone configuration is invalid: ", result.error());
    return;
  }

  // Register ROS interfaces.
  drone_pub_ = createPublisher<Drone>(topic::kDrone, true, true);

  // Defer the initial publication until the executor starts processing callbacks.
  initial_publish_timer_ = createTimer(0s, &self::initialPublishTimerCb, this);
}

void DroneServerNode::initialPublishTimerCb()
{
  initial_publish_timer_->cancel();

  auto drone_msg = std::make_unique<Drone>(drone_);
  drone_pub_->publish(std::move(drone_msg));
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::DroneServerNode)

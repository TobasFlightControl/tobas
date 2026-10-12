// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_gamepad_core/gamepad_rc_input.hpp>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_msgs_adapter/rc_input.hpp>
#include <tobas_node/node.hpp>

using namespace std::chrono_literals;

namespace tobas
{
namespace gamepad
{
/** Read gamepad input and publish it as RC input messages. */
class RcInputPublisher : public BaseNode
{
  using self = RcInputPublisher;
  using super = BaseNode;

public:
  explicit RcInputPublisher(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());

  void initialize();

private:
  void publishFromState(const GamepadState& _state);
  void timerCallback();

  std::string device_path_;

  GamepadDriver gamepad_;

  ros2::PublisherPtr<tobas_msgs::RCInput> publisher_;
  ros2::TimerPtr initialize_timer_;
  ros2::TimerPtr main_timer_;
};

RcInputPublisher::RcInputPublisher(const rclcpp::NodeOptions& _options)
  : super("rc_input_publisher", nodeOptions_Default(_options))
{
  device_path_ = getStringParam("device_path");

  publisher_ = createPublisher<tobas_msgs::RCInput>(topic::kRcInput);
  initialize_timer_ = createWallTimer(1s, &self::initialize, this);
}

void RcInputPublisher::initialize()
{
  if (const auto result = gamepad_.initialize(device_path_); !result) {
    TOBAS_WARN("Failed to initialize gamepad driver: ", result.error(), ". Retrying...");
    return;
  }

  initialize_timer_->cancel();
  main_timer_ = createWallTimer(10ms, &self::timerCallback, this);

  TOBAS_INFO("Gamepad driver has been initialized.");
}

void RcInputPublisher::timerCallback()
{
  const auto state = gamepad_.read();
  if (!state) {
    TOBAS_WARN("Failed to read gamepad RC input: ", state.error());
    return;
  }

  publishFromState(*state);
}

void RcInputPublisher::publishFromState(const GamepadState& _state)
{
  auto msg = std::make_unique<tobas_msgs::RCInput>();
  msg->header.stamp = now();
  msg->status = _state.ok ? tobas_msgs::msg::RCInput::STATUS_OK : tobas_msgs::msg::RCInput::STATUS_OTHER;
  msg->roll = _state.roll;
  msg->pitch = _state.pitch;
  msg->throttle = _state.throttle;
  msg->yaw = _state.yaw;
  msg->mode = _state.mode;
  msg->sub_mode = _state.sub_mode;
  msg->enable = _state.enable;
  msg->kill = _state.kill;
  msg->gpsw = _state.gpsw;
  publisher_->publish(std::move(msg));
}
}  // namespace gamepad
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::gamepad::RcInputPublisher)

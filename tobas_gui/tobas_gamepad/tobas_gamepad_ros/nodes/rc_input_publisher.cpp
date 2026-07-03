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
/**
 * @brief Read gamepad input and publish it as RC input messages.
 */
class RcInputPublisher : public BaseNode
{
  using self = RcInputPublisher;
  using super = BaseNode;

public:
  explicit RcInputPublisher(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());

  void initialize();

private:
  void publishFromState(const GamepadRcInputState& _state);
  void timerCallback();

  std::string device_path_;
  GamepadRcInput gamepad_;
  ros2::PublisherPtr<tobas_msgs::RCInput> publisher_;
  ros2::TimerPtr initialize_timer_;
  ros2::TimerPtr timer_;
};

RcInputPublisher::RcInputPublisher(const rclcpp::NodeOptions& _options)
  : super("rc_input_publisher", nodeOptions_Default(_options))
{
  device_path_ = getStringParam("device_path", "");
  if (device_path_.empty()) {
    TOBAS_WARN("No device path specified. This node will not work.");
    return;
  }

  publisher_ = createPublisher<tobas_msgs::RCInput>(topic::kRcInput);
  initialize_timer_ = createWallTimer(3s, &self::initialize, this);
}

void RcInputPublisher::initialize()
{
  if (!gamepad_.initialize(device_path_)) {
    TOBAS_WARN("Failed to initialize gamepad RC input with device \"", device_path_, "\". Retrying...");
    return;
  }

  initialize_timer_->cancel();
  timer_ = createWallTimer(10ms, &self::timerCallback, this);

  TOBAS_INFO("Initialized gamepad RC input with device \"", device_path_, "\".");
}

void RcInputPublisher::timerCallback()
{
  GamepadRcInputState state;
  if (!gamepad_.read(state)) {
    TOBAS_WARN("Failed to read gamepad RC input.");
    return;
  }

  publishFromState(state);
}

void RcInputPublisher::publishFromState(const GamepadRcInputState& _state)
{
  auto msg = std::make_unique<tobas_msgs::RCInput>();
  msg->header.stamp = now();
  msg->ok = _state.ok;
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

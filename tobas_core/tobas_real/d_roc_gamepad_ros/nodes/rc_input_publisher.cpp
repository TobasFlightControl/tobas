// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <d_roc_gamepad/gamepad_rc_input.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_msgs_adapter/rc_input.hpp>
#include <tobas_node/node.hpp>

using namespace std::chrono_literals;

namespace tobas
{
namespace d_roc_gamepad
{
/**
 * @brief ゲームパッド入力を読み取り，RC入力メッセージとして発行する．
 */
class RcInputPublisher : public BaseNode
{
  using self = RcInputPublisher;
  using super = BaseNode;

public:
  explicit RcInputPublisher(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());

  void initialize();

private:
  void publishFromState(const driver::GamepadRcInputState& _state);
  void timerCallback();

  std::string device_path_;
  driver::GamepadRcInput gamepad_;
  ros2::PublisherPtr<tobas_msgs::RCInput> publisher_;
  ros2::TimerPtr initialize_timer_;
  ros2::TimerPtr timer_;
};

RcInputPublisher::RcInputPublisher(const rclcpp::NodeOptions& _options)
  : super("rc_input_publisher", nodeOptions_Default(_options))
{
  device_path_ = getStringParam("device_path", "");
  if (device_path_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No device path specified. This node will not work.");
    return;
  }

  publisher_ = createPublisher<tobas_msgs::RCInput>(topic::kRcInput);
  initialize_timer_ = createWallTimer(3s, &self::initialize, this);
}

void RcInputPublisher::initialize()
{
  if (!this->gamepad_.initialize(this->device_path_)) {
    RCLCPP_WARN(
      this->get_logger(), "Failed to initialize gamepad RC input with device \"%s\". Retrying...", device_path_.c_str());
    return;
  }

  initialize_timer_->cancel();
  timer_ = createWallTimer(10ms, &self::timerCallback, this);

  RCLCPP_INFO(this->get_logger(), "Initialized gamepad RC input with device \"%s\".", device_path_.c_str());
}

void RcInputPublisher::timerCallback()
{
  driver::GamepadRcInputState state;
  if (!this->gamepad_.read(state)) {
    RCLCPP_WARN(this->get_logger(), "Failed to read gamepad RC input.");
    return;
  }

  this->publishFromState(state);
}

void RcInputPublisher::publishFromState(const driver::GamepadRcInputState& _state)
{
  auto msg = std::make_unique<tobas_msgs::RCInput>();
  msg->header.stamp = now();
  msg->ok = _state.ok;
  msg->roll = _state.roll;
  msg->pitch = _state.pitch;
  msg->throttle = _state.throttle;
  msg->yaw = _state.yaw;
  msg->mode = static_cast<FlightMode>(_state.mode);
  msg->sub_mode = _state.sub_mode;
  msg->enable = _state.enable;
  msg->kill = _state.kill;
  msg->gpsw = _state.gpsw;
  publisher_->publish(std::move(msg));
}

}  // namespace d_roc_gamepad
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::d_roc_gamepad::RcInputPublisher)

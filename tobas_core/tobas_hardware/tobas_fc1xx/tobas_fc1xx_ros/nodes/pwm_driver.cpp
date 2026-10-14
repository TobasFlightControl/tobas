// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_fc1xx_core/pwm.hpp>
#include <tobas_hardware_common/constants.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs/msg/pwm_array.hpp>

namespace tobas::fc1xx
{
class PwmDriverNode : public BaseNode
{
  using self = PwmDriverNode;
  using super = BaseNode;

public:
  explicit PwmDriverNode(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());

private:
  PWM pwm_;
  ros2::SubscriberPtr<tobas_msgs::msg::PwmArray> pwms_sub_;
  ros2::TimerPtr initialize_timer_;

  void initialize();
  void pwmsCb(const tobas_msgs::msg::PwmArray::ConstSharedPtr& _pwms);
};

PwmDriverNode::PwmDriverNode(const rclcpp::NodeOptions& _options)
  : super("fc1xx_pwm_driver", nodeOptions_Default(_options))
{
  initialize_timer_ = createWallTimer(hardware::kRetryInitializationInterval, &self::initialize, this);
}

void PwmDriverNode::initialize()
{
  if (!pwm_.initialize()) {
    TOBAS_ERROR("Failed to initialize PWM driver. Retrying...");
    return;
  }

  pwms_sub_ = createSubscriber(topic::kPwmCmd, &self::pwmsCb, this);

  initialize_timer_->cancel();
}

void PwmDriverNode::pwmsCb(const tobas_msgs::msg::PwmArray::ConstSharedPtr& _pwms)
{
  // Set PWM periods of each channel.
  for (const auto& elem : _pwms->pwms) {
    if (elem.channel >= PWM::kChannelSize) {
      TOBAS_ERROR("PWM channel ", elem.channel, " does not exist.");
      continue;
    }
    pwm_.setPeriod(elem.channel, elem.period);
  }

  // Send PWM pwms.
  if (!pwm_.transfer()) {
    TOBAS_ERROR("Failed to send PWM command.");
  }
}
}  // namespace tobas::fc1xx

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::fc1xx::PwmDriverNode)

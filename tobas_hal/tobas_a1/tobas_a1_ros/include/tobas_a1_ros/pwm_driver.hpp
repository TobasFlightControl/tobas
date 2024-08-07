#pragma once

#include <tobas_node/node.hpp>
#include <tobas_msgs/PwmArray.h>

#include <tobas_a1_core/pwm.hpp>

namespace a1
{
class PWMDriver : public tobas::BaseNode
{
  using self = PWMDriver;
  using super = tobas::BaseNode;

public:
  explicit PWMDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  PWM pwm_;
  SubscriberPtr<> pwms_sub_;

  void pwmsCb(const tobas_msgs::PwmArray::ConstSharedPtr& pwms);
};
}  // namespace a1

#pragma once

#include <tobas_tools/node.hpp>
#include <tobas_msgs/PwmArray.h>

#include <tobas_a1_core/pwm.hpp>

namespace a1
{
class PWMDriver : public tobas::BaseNode
{
  using self = PWMDriver;
  using super = tobas::BaseNode;

public:
  explicit PWMDriver(, const std::string& name = rclcpp::this_node::getName());

private:
  PWM pwm_;
  rclcpp::Subscriber pwms_sub_;

  void pwmsCb(const tobas_msgs::PwmArrayConstPtr& pwms);
};
}  // namespace a1

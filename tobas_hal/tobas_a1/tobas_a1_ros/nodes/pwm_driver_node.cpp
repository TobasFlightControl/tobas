#include <tobas_math/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/pwm_array.hpp>

#include <tobas_a1_core/pwm.hpp>

using namespace std;

class PWMDriverNode : public tobas::BaseNode
{
  using self = PWMDriverNode;
  using super = tobas::BaseNode;

public:
  explicit PWMDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  a1::PWM pwm_;
  SubscriberPtr<tobas_msgs::msg::PwmArray> pwms_sub_;

  void pwmsCb(const tobas_msgs::msg::PwmArray::ConstSharedPtr& pwms);
};

PWMDriverNode::PWMDriverNode(const rclcpp::NodeOptions& options) : super("a1_pwm_driver", options)
{
  if (!pwm_.initialize())
    TOBAS_EXIT("Failed to initialize PWM driver.");

  pwms_sub_ = createSubscriber(tobas::kPwmCmdTopic, &self::pwmsCb, this);
}

void PWMDriverNode::pwmsCb(const tobas_msgs::msg::PwmArray::ConstSharedPtr& pwms)
{
  // Set PWM periods of each channel
  for (const auto& elem : pwms->pwms)
  {
    if (elem.channel >= a1::PWM::kChannelSize)
    {
      TOBAS_ERROR("PWM channel ", elem.channel, " does not exist.");
      continue;
    }

    TOBAS_ASSERT(pwm_.setPeriod(elem.channel, elem.period));
  }

  // Send PWM pwms
  if (!pwm_.transfer())
    TOBAS_ERROR("Failed to send PWM command.");
}

RCLCPP_COMPONENTS_REGISTER_NODE(PWMDriverNode)

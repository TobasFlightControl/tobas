#include <tobas_math/core.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/pwm_array.hpp>

#include <tobas_t1_core/pwm.hpp>

#include "./common.hpp"

using namespace std;

class PwmDriverNode : public tobas::BaseNode
{
  using self = PwmDriverNode;
  using super = tobas::BaseNode;

public:
  explicit PwmDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  t1::PWM pwm_;
  ros2::SubscriberPtr<tobas_msgs::msg::PwmArray> pwms_sub_;
  ros2::TimerPtr initialize_timer_;

  void initialize();
  void pwmsCb(const tobas_msgs::msg::PwmArray::ConstSharedPtr& pwms);
};

PwmDriverNode::PwmDriverNode(const rclcpp::NodeOptions& options) : super("t1_pwm_driver", options)
{
  initialize_timer_ = createWallTimer(t1::kRetryInitializationInterval, &self::initialize, this);
}

void PwmDriverNode::initialize()
{
  if (!pwm_.initialize())
  {
    TOBAS_ERROR("Failed to initialize PWM driver. Retrying...");
    return;
  }

  pwms_sub_ = createSubscriber(tobas::kPwmCmdTopic, &self::pwmsCb, this);

  initialize_timer_->cancel();
}

void PwmDriverNode::pwmsCb(const tobas_msgs::msg::PwmArray::ConstSharedPtr& pwms)
{
  // Set PWM periods of each channel
  for (const auto& elem : pwms->pwms)
  {
    if (elem.channel >= t1::PWM::kChannelSize)
    {
      TOBAS_ERROR("PWM channel ", elem.channel, " does not exist.");
      continue;
    }

    if (!pwm_.setPeriod(elem.channel, elem.period))
    {
      TOBAS_ERROR("PWM command of channel ", elem.channel, " is rejected.");
      continue;
    }
  }

  // Send PWM pwms
  if (!pwm_.transfer())
    TOBAS_ERROR("Failed to send PWM command.");
}

RCLCPP_COMPONENTS_REGISTER_NODE(PwmDriverNode)

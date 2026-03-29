#include <tobas_constants/ros_interface.hpp>
#include <tobas_fc1xx_core/pwm.hpp>
#include <tobas_math/core.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs/msg/pwm_array.hpp>

#include "./common.hpp"

namespace tobas
{
namespace fc1xx
{
class PwmDriverNode : public BaseNode
{
  using self = PwmDriverNode;
  using super = BaseNode;

public:
  explicit PwmDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  PWM pwm_;
  ros2::SubscriberPtr<tobas_msgs::msg::PwmArray> pwms_sub_;
  ros2::TimerPtr initialize_timer_;

  void initialize();
  void pwmsCb(const tobas_msgs::msg::PwmArray::ConstSharedPtr& pwms);
};

PwmDriverNode::PwmDriverNode(const rclcpp::NodeOptions& options)
  : super("fc1xx_pwm_driver", nodeOptions_Default(options))
{
  initialize_timer_ = createWallTimer(kRetryInitializationInterval, &self::initialize, this);
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

void PwmDriverNode::pwmsCb(const tobas_msgs::msg::PwmArray::ConstSharedPtr& pwms)
{
  // Set PWM periods of each channel
  for (const auto& elem : pwms->pwms) {
    if (elem.channel >= PWM::kChannelSize) {
      TOBAS_ERROR("PWM channel ", elem.channel, " does not exist.");
      continue;
    }

    if (!pwm_.setPeriod(elem.channel, elem.period)) {
      TOBAS_ERROR("PWM command of channel ", elem.channel, " was rejected.");
      continue;
    }
  }

  // Send PWM pwms
  if (!pwm_.transfer()) {
    TOBAS_ERROR("Failed to send PWM command.");
  }
}
}  // namespace fc1xx
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::fc1xx::PwmDriverNode)

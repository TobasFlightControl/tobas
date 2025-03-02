#include <tobas_path_tools/join.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/engine_throttle.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>
#include <tobas_msgs/srv/set_arm.hpp>

#include <tobas_gazebo_common/constants.hpp>

using namespace std;

class EngineThrottleCommandHandlerNode : public tobas::BaseNode
{
  using self = EngineThrottleCommandHandlerNode;
  using super = tobas::BaseNode;

public:
  explicit EngineThrottleCommandHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  bool is_armed_ = false;
  tobas_msgs::msg::PreArmCheck::ConstSharedPtr prearm_check_;

  ros2::PublisherPtr<tobas_msgs::msg::EngineThrottle> throttle_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::Arming> arming_pub_;

  ros2::SubscriberPtr<tobas_msgs::msg::EngineThrottle> throttle_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::PreArmCheck> prearm_check_sub_;

  ros2::ServiceServerPtr<tobas_msgs::srv::SetArm> set_arm_ss_;

  ros2::TimerPtr publish_arm_status_timer_;

  void publishArming();

  void throttleCb(const tobas_msgs::msg::EngineThrottle::ConstSharedPtr& throttle_in);
  void preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check);

  void setArmCb(
    const tobas_msgs::srv::SetArm::Request::ConstSharedPtr& req,
    const tobas_msgs::srv::SetArm::Response::SharedPtr& res);
};

EngineThrottleCommandHandlerNode::EngineThrottleCommandHandlerNode(const rclcpp::NodeOptions& options)
  : super("gazebo_engine_throttle_command_handler", options)
{
  throttle_pub_ = createPublisher<tobas_msgs::msg::EngineThrottle>(gazebo::kEngineThrottleCmdTopic);
  arming_pub_ = createPublisher<tobas_msgs::msg::Arming>(tobas::kArmingTopic);

  throttle_sub_ = createSubscriber(tobas::kEngineThrottleCmdTopic, &self::throttleCb, this);
  prearm_check_sub_ = createSubscriber(tobas::kPreArmCheckTopic, &self::preArmCheckCb, this);

  set_arm_ss_ = createService<tobas_msgs::srv::SetArm>(tobas::kSetArmSrv, &self::setArmCb, this);

  publish_arm_status_timer_ = createTimer(tobas::kPublishArmingPeriod, &self::publishArming, this);
}

void EngineThrottleCommandHandlerNode::publishArming()
{
  auto arming_msg = std::make_unique<tobas_msgs::msg::Arming>();
  arming_msg->header.stamp = get_clock()->now();
  arming_msg->data = is_armed_;
  arming_pub_->publish(move(arming_msg));
}

void EngineThrottleCommandHandlerNode::throttleCb(const tobas_msgs::msg::EngineThrottle::ConstSharedPtr& throttle_in)
{
  // アームしてないならスロットルコマンドをブロック
  if (!is_armed_)
  {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Command is ignored because the rotors are disarmed.");
    return;
  }

  auto throttle_out = std::make_unique<tobas_msgs::msg::EngineThrottle>(*throttle_in);
  throttle_pub_->publish(move(throttle_out));
}

void EngineThrottleCommandHandlerNode::preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check)
{
  prearm_check_ = prearm_check;
}

void EngineThrottleCommandHandlerNode::setArmCb(
  const tobas_msgs::srv::SetArm::Request::ConstSharedPtr& req,
  const tobas_msgs::srv::SetArm::Response::SharedPtr& res)
{
  TOBAS_INFO("Set arm requested.");

  if (!is_armed_ && req->arming)
  {
    if (!req->ignore_prearm_check)
    {
      if (prearm_check_ == nullptr)
      {
        res->success = false;
        res->message = "Pre-arm check status is not received yet.";
        return;
      }

      if (!prearm_check_->ok)
      {
        res->success = false;
        res->message = "Pre-arm check failed.";
        return;
      }
    }

    is_armed_ = true;
    publishArming();
  }
  else if (is_armed_ && !req->arming)
  {
    is_armed_ = false;
    publishArming();
  }

  res->success = true;
}

RCLCPP_COMPONENTS_REGISTER_NODE(EngineThrottleCommandHandlerNode)

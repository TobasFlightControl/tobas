#include <std_msgs/msg/bool.hpp>

#include <tobas_math/core.hpp>
#include <tobas_algorithm/core.hpp>
#include <tobas_std_tools/vector.hpp>
#include <tobas_std_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>

#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>
#include <tobas_msgs/srv/enable_rc_output.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_drone_msgs_adapter/Drone.hpp>

using namespace std;

class RotorControllerNode : public tobas::BaseNode
{
  static constexpr double kCmdWarnPeriod = 1.;  // [s]
  static constexpr auto kPublishArmingPeriod = 1s;

  using self = RotorControllerNode;
  using super = tobas::BaseNode;

public:
  explicit RotorControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  bool is_armed_ = false;
  tobas::Drone::ConstSharedPtr drone_;
  tobas_msgs::msg::PreArmCheck::ConstSharedPtr prearm_check_;

  // PubSub
  ros2::PublisherPtr<tobas_msgs::msg::RotorSpeedArray> tar_speeds_pub_;
  ros2::PublisherPtr<std_msgs::msg::Bool> arming_pub_;
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorThrustArray> tar_thrusts_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::PreArmCheck> prearm_check_sub_;

  // Service
  ros2::ServiceServerPtr<tobas_msgs::srv::SetArm> set_arm_ss_;
  ros2::ServiceClientPtr<tobas_msgs::srv::EnableRCOutput> enable_rcout_sc_;

  // Timer
  ros2::TimerPtr publish_arm_status_timer_;

  bool enableRCOutputs(const bool& enable);
  void publishArming();

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void thrustsCmdCb(const tobas_msgs::msg::RotorThrustArray::ConstSharedPtr& tar_thrusts_msg);
  void preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check);

  void setArmCb(
    const tobas_msgs::srv::SetArm::Request::ConstSharedPtr& req,
    const tobas_msgs::srv::SetArm::Response::SharedPtr& res);

  void publishArmThrotTimerCb();
};

RotorControllerNode::RotorControllerNode(const rclcpp::NodeOptions& options) : super("rotor_controller", options)
{
  tar_speeds_pub_ = createPublisher<tobas_msgs::msg::RotorSpeedArray>(tobas::kRotorSpeedsCmdTopic);
  arming_pub_ = createPublisher<std_msgs::msg::Bool>(tobas::kArmingTopic);

  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  tar_thrusts_sub_ = createSubscriber(tobas::kRotorThrustsCmdTopic, &self::thrustsCmdCb, this);
  prearm_check_sub_ = createSubscriber(tobas::kPreArmCheckTopic, &self::preArmCheckCb, this);

  set_arm_ss_ = createService<tobas_msgs::srv::SetArm>(tobas::kSetArmSrv, &self::setArmCb, this);
  enable_rcout_sc_ = create_client<tobas_msgs::srv::EnableRCOutput>(tobas::kEnableRcOutputSrv);

  publish_arm_status_timer_ = createTimer(kPublishArmingPeriod, &self::publishArming, this);
}

bool RotorControllerNode::enableRCOutputs(const bool& enable)
{
  if (!enable_rcout_sc_->service_is_ready())
  {
    TOBAS_ERROR("\"", tobas::kEnableRcOutputSrv, "\" is not ready.");
    return false;
  }

  for (const auto& rotor : drone_->rotors)
  {
    const auto req = std::make_shared<tobas_msgs::srv::EnableRCOutput::Request>();
    req->channel = rotor.channel;
    req->enable = enable;

    enable_rcout_sc_->async_send_request(req);  // TODO: 結果を確認
  }

  return true;
}

void RotorControllerNode::publishArming()
{
  auto arming_msg = std::make_unique<std_msgs::msg::Bool>();
  arming_msg->data = is_armed_;
  arming_pub_->publish(move(arming_msg));
}

void RotorControllerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = drone;
}

void RotorControllerNode::thrustsCmdCb(const tobas_msgs::msg::RotorThrustArray::ConstSharedPtr& tar_thrusts_msg)
{
  if (!is_armed_)
    return;

  if (drone_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(kCmdWarnPeriod, "Command is ignored because drone configuration has not been received yet.");
    return;
  }

  // Create target speeds message
  auto tar_speeds_msg = std::make_unique<tobas_msgs::msg::RotorSpeedArray>();
  tar_speeds_msg->header = tar_thrusts_msg->header;

  // Convert target thrusts to target speeds
  for (const auto& tar_thrust_msg : tar_thrusts_msg->thrusts)
  {
    const auto& channel = tar_thrust_msg.channel;
    const auto& tar_thrust = tar_thrust_msg.thrust;

    tar_speeds_msg->speeds.emplace_back();
    tar_speeds_msg->speeds.back().channel = channel;

    if (tar_thrust >= 0.)
    {
      tar_speeds_msg->speeds.back().speed = drone_->rotSpeedFromThrust(channel, tar_thrust_msg.thrust);
    }
    else
    {
      TOBAS_WARN_THROTTLE(kCmdWarnPeriod, "Negative thrust is commanded on CH", channel, ": ", tar_thrust, " < 0 [N]");
      tar_speeds_msg->speeds.back().speed = 0.;
    }
  }

  // Publish throttle commands
  tar_speeds_pub_->publish(move(tar_speeds_msg));
}

void RotorControllerNode::preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check)
{
  prearm_check_ = prearm_check;
}

void RotorControllerNode::setArmCb(
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
        TOBAS_ERROR(res->message);
        return;
      }

      if (!prearm_check_->ok)
      {
        res->success = false;
        res->message = "Pre-arm check failed.";
        TOBAS_ERROR(res->message);
        return;
      }
    }

    if (!enableRCOutputs(true))
    {
      res->success = false;
      res->message = "Failed to enable rotors.";
      TOBAS_ERROR(res->message);
      return;
    }

    is_armed_ = true;
    publishArming();
  }
  else if (is_armed_ && !req->arming)
  {
    if (!enableRCOutputs(false))
    {
      res->success = false;
      res->message = "Failed to disable rotors.";
      TOBAS_ERROR(res->message);
      return;
    }

    is_armed_ = false;
    publishArming();
  }

  res->success = true;
}

RCLCPP_COMPONENTS_REGISTER_NODE(RotorControllerNode)

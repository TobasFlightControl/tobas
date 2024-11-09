#include <std_msgs/msg/bool.hpp>

#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_drone_msgs_adapter/Drone.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>
#include <tobas_msgs/srv/set_arm.hpp>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_msgs/msg/throttle.hpp>

using namespace std;

class RotorCommandHandlerNode : public tobas::BaseNode
{
  using self = RotorCommandHandlerNode;
  using super = tobas::BaseNode;

public:
  explicit RotorCommandHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  bool is_armed_ = false;
  tobas::Drone::ConstSharedPtr drone_;
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;
  tobas_msgs::msg::PreArmCheck::ConstSharedPtr prearm_check_;

  map<uint8_t, ros2::PublisherPtr<tobas_gazebo_msgs::msg::Throttle>> throttle_pubs_;
  ros2::PublisherPtr<std_msgs::msg::Bool> arming_pub_;

  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Battery> battery_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorSpeedArray> tar_speeds_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::PreArmCheck> prearm_check_sub_;

  ros2::ServiceServerPtr<tobas_msgs::srv::SetArm> set_arm_ss_;

  ros2::TimerPtr publish_arm_status_timer_;

  void publishArming();

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void targetSpeedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& tar_speeds);
  void preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check);

  void setArmCb(
    const tobas_msgs::srv::SetArm::Request::ConstSharedPtr& req,
    const tobas_msgs::srv::SetArm::Response::SharedPtr& res);
};

RotorCommandHandlerNode::RotorCommandHandlerNode(const rclcpp::NodeOptions& options)
  : super("gazebo_rotor_command_handler", options)
{
  arming_pub_ = createPublisher<std_msgs::msg::Bool>(tobas::kArmingTopic);

  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  battery_sub_ = createSubscriber(tobas::kBatteryTopic, &self::batteryCb, this);
  tar_speeds_sub_ = createSubscriber(tobas::kRotorSpeedsCmdTopic, &self::targetSpeedsCb, this);
  prearm_check_sub_ = createSubscriber(tobas::kPreArmCheckTopic, &self::preArmCheckCb, this);

  set_arm_ss_ = createService<tobas_msgs::srv::SetArm>(tobas::kSetArmSrv, &self::setArmCb, this);

  publish_arm_status_timer_ = createTimer(tobas::kPublishArmingPeriod, &self::publishArming, this);
}

void RotorCommandHandlerNode::publishArming()
{
  auto arming_msg = std::make_unique<std_msgs::msg::Bool>();
  arming_msg->data = is_armed_;
  arming_pub_->publish(move(arming_msg));
}

void RotorCommandHandlerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  throttle_pubs_.clear();
  for (const auto& rotor : drone->rotors)
  {
    const auto topic = string(gazebo::kThrottleTopicPrefix) + "_" + to_string(rotor.channel);
    throttle_pubs_[rotor.channel] = createPublisher<tobas_gazebo_msgs::msg::Throttle>(topic);
  }

  drone_ = drone;

  TOBAS_INFO("Gazego rotor command handler is initialized.");
}

void RotorCommandHandlerNode::batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery)
{
  battery_ = battery;
}

void RotorCommandHandlerNode::targetSpeedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& tar_speeds)
{
  if (drone_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Drone message is not received yet.");
    return;
  }
  if (battery_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Battery message is not received yet.");
    return;
  }

  for (const auto& speed : tar_speeds->speeds)
  {
    // Check channel
    if (!throttle_pubs_.contains(speed.channel))
    {
      TOBAS_ERROR("The drone does not have rotor channel ", speed.channel, ".");
      return;
    }

    // Create throttle message
    auto throttle = std::make_unique<tobas_gazebo_msgs::msg::Throttle>();
    throttle->header = tar_speeds->header;
    throttle->data = drone_->throttleFromRotSpeed(speed.channel, speed.speed, battery_->voltage);  // FF項のみ

    // Publish throttle message
    throttle_pubs_.at(speed.channel)->publish(move(throttle));
  }
}

void RotorCommandHandlerNode::preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check)
{
  prearm_check_ = prearm_check;
}

void RotorCommandHandlerNode::setArmCb(
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

RCLCPP_COMPONENTS_REGISTER_NODE(RotorCommandHandlerNode)

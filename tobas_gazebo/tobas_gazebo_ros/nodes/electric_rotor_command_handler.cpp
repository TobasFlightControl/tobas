#include <tobas_path_tools/join.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>
#include <tobas_msgs/srv/set_arm.hpp>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_msgs/msg/throttle.hpp>

using namespace std;

class ElectricRotorCommandHandlerNode : public tobas::BaseNode
{
  using self = ElectricRotorCommandHandlerNode;
  using super = tobas::BaseNode;

public:
  explicit ElectricRotorCommandHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  bool is_armed_ = false;
  tobas::ElectricPropulsionSystemConfig::ConstSharedPtr eprop_;
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;
  tobas_msgs::msg::PreArmCheck::ConstSharedPtr prearm_check_;

  map<string, ros2::PublisherPtr<tobas_gazebo_msgs::msg::Throttle>> throttle_pubs_;
  ros2::PublisherPtr<tobas_msgs::msg::Arming> arming_pub_;

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

ElectricRotorCommandHandlerNode::ElectricRotorCommandHandlerNode(const rclcpp::NodeOptions& options)
  : super("gazebo_electric_rotor_command_handler", options)
{
  arming_pub_ = createPublisher<tobas_msgs::msg::Arming>(tobas::kArmingTopic);

  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  battery_sub_ = createSubscriber(tobas::kBatteryTopic, &self::batteryCb, this);
  tar_speeds_sub_ = createSubscriber(tobas::kRotorSpeedsCmdTopic, &self::targetSpeedsCb, this);
  prearm_check_sub_ = createSubscriber(tobas::kPreArmCheckTopic, &self::preArmCheckCb, this);

  set_arm_ss_ = createService<tobas_msgs::srv::SetArm>(tobas::kSetArmSrv, &self::setArmCb, this);

  publish_arm_status_timer_ = createTimer(tobas::kPublishArmingPeriod, &self::publishArming, this);
}

void ElectricRotorCommandHandlerNode::publishArming()
{
  auto arming_msg = std::make_unique<tobas_msgs::msg::Arming>();
  arming_msg->header.stamp = get_clock()->now();
  arming_msg->data = is_armed_;
  arming_pub_->publish(move(arming_msg));
}

void ElectricRotorCommandHandlerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  if (drone->prop == nullptr)
    return;

  if (drone->prop->type() != tobas::propulsion_system_t::ELECTRIC)
  {
    TOBAS_WARN("Only supports electric propulsion system.");
    return;
  }

  eprop_ = boost::polymorphic_pointer_downcast<tobas::ElectricPropulsionSystemConfig>(drone->prop);

  throttle_pubs_.clear();
  for (const auto& [link_name, _] : eprop_->rotors)
  {
    const auto topic = path::join(gazebo::kRotorThrottleCmdTopicNS, link_name);
    throttle_pubs_[link_name] = createPublisher<tobas_gazebo_msgs::msg::Throttle>(topic);
  }
}

void ElectricRotorCommandHandlerNode::batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery)
{
  battery_ = battery;
}

void ElectricRotorCommandHandlerNode::targetSpeedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& tar_speeds)
{
  if (!is_armed_)
  {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Command is ignored because the rotors are disarmed.");
    return;
  }

  if (eprop_ == nullptr)
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
    // Check link name
    if (!throttle_pubs_.contains(speed.link_name))
    {
      TOBAS_ERROR("Electric rotor \"" + speed.link_name + "\" does not exist.");
      return;
    }

    // Create throttle message
    auto throttle = std::make_unique<tobas_gazebo_msgs::msg::Throttle>();
    throttle->header = tar_speeds->header;
    throttle->data = eprop_->getRotor(speed.link_name)->throttleFromSpeed(speed.speed, battery_->voltage);  // FF項のみ

    // Publish throttle message
    throttle_pubs_.at(speed.link_name)->publish(move(throttle));
  }
}

void ElectricRotorCommandHandlerNode::preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check)
{
  prearm_check_ = prearm_check;
}

void ElectricRotorCommandHandlerNode::setArmCb(
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

RCLCPP_COMPONENTS_REGISTER_NODE(ElectricRotorCommandHandlerNode)

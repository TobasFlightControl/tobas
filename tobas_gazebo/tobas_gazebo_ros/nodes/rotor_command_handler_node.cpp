#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_drone_msgs_adapter/Drone.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/srv/enable_rc_output.hpp>

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
  tobas::Drone::ConstSharedPtr drone_;
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;

  map<uint8_t, ros2::PublisherPtr<tobas_gazebo_msgs::msg::Throttle>> throttle_pubs_;
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Battery> battery_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorSpeedArray> speeds_sub_;
  ros2::ServiceServerPtr<tobas_msgs::srv::EnableRCOutput> enable_rcout_srv_;

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void rotorSpeedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& speeds);
  void enableRCOutputCb(
    const tobas_msgs::srv::EnableRCOutput::Request::ConstSharedPtr& req,
    const tobas_msgs::srv::EnableRCOutput::Response::SharedPtr& res);
};

RotorCommandHandlerNode::RotorCommandHandlerNode(const rclcpp::NodeOptions& options)
  : super("gazebo_rotor_command_handler", options)
{
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  battery_sub_ = createSubscriber(tobas::kBatteryTopic, &self::batteryCb, this);
  speeds_sub_ = createSubscriber(tobas::kRotorSpeedsCmdTopic, &self::rotorSpeedsCb, this);

  enable_rcout_srv_ =
    createService<tobas_msgs::srv::EnableRCOutput>(tobas::kEnableRcOutputSrv, &self::enableRCOutputCb, this);
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

void RotorCommandHandlerNode::rotorSpeedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& speeds)
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

  for (const auto& speed : speeds->speeds)
  {
    // Check channel
    if (!throttle_pubs_.contains(speed.channel))
    {
      TOBAS_ERROR("The drone does not have rotor channel ", speed.channel, ".");
      return;
    }

    // Create throttle message
    auto throttle = std::make_unique<tobas_gazebo_msgs::msg::Throttle>();
    throttle->header = speeds->header;
    throttle->data = drone_->throttleFromRotSpeed(speed.channel, speed.speed, battery_->voltage);  // FF項のみ

    // Publish throttle message
    throttle_pubs_.at(speed.channel)->publish(move(throttle));
  }
}

void RotorCommandHandlerNode::enableRCOutputCb(
  const tobas_msgs::srv::EnableRCOutput::Request::ConstSharedPtr& req,
  const tobas_msgs::srv::EnableRCOutput::Response::SharedPtr& res)
{
  if (!throttle_pubs_.contains(req->channel))
  {
    res->success = false;
    res->message = "The drone does not have rotor channel " + to_string(req->channel) + ".";
    return;
  }

  // TODO: ちゃんとサービスを実装する

  res->success = true;
  res->message.clear();
  return;
}

RCLCPP_COMPONENTS_REGISTER_NODE(RotorCommandHandlerNode)

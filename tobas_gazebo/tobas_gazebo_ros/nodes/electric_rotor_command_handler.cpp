#include <tobas_constants/constants.hpp>
#include <tobas_gazebo_common/constants.hpp>
#include <tobas_node/node.hpp>
#include <tobas_path_tools/join.hpp>

#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_gazebo_msgs/msg/throttle.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>

using namespace std;

class ElectricRotorCommandHandlerNode : public tobas::BaseNode
{
  using self = ElectricRotorCommandHandlerNode;
  using super = tobas::BaseNode;

public:
  explicit ElectricRotorCommandHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::ElectricPropulsionSystemConfig::ConstSharedPtr eprop_;
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;

  map<string, ros2::PublisherPtr<tobas_gazebo_msgs::msg::Throttle>> throttle_pubs_;

  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Battery> battery_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorSpeedArray> tar_speeds_sub_;

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void targetSpeedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& tar_speeds);
};

ElectricRotorCommandHandlerNode::ElectricRotorCommandHandlerNode(const rclcpp::NodeOptions& options)
  : super("gazebo_electric_rotor_command_handler", options)
{
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
}

void ElectricRotorCommandHandlerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  if (!drone->prop) {
    return;
  }

  if (drone->prop->type() != tobas::propulsion_system_t::ELECTRIC) {
    return;
  }

  eprop_ = boost::polymorphic_pointer_downcast<tobas::ElectricPropulsionSystemConfig>(drone->prop);

  // Register publishers
  throttle_pubs_.clear();
  for (const auto& [link_name, _] : eprop_->rotors) {
    const auto topic = path::join(gazebo::kRotorThrottleCmdTopicNS, link_name);
    throttle_pubs_[link_name] = createPublisher<tobas_gazebo_msgs::msg::Throttle>(topic);
  }

  // Register subscribers
  battery_sub_ = createSubscriber(tobas::kBatteryTopic, &self::batteryCb, this);
  tar_speeds_sub_ = createSubscriber(tobas::kRotorSpeedsCmdTopic, &self::targetSpeedsCb, this);
}

void ElectricRotorCommandHandlerNode::batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery)
{
  battery_ = battery;
}

void ElectricRotorCommandHandlerNode::targetSpeedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& tar_speeds)
{
  if (!eprop_) {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Drone message is not received yet.");
    return;
  }
  if (!battery_) {
    TOBAS_WARN_THROTTLE(tobas::kTypicalWarnPeriod, "Battery message is not received yet.");
    return;
  }

  for (const auto& speed : tar_speeds->speeds) {
    // Check link name
    if (!throttle_pubs_.contains(speed.link_name)) {
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

RCLCPP_COMPONENTS_REGISTER_NODE(ElectricRotorCommandHandlerNode)

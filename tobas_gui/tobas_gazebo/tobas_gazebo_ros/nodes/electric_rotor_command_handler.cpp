// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <boost/polymorphic_pointer_cast.hpp>

#include <tobas_constants/time.hpp>
#include <tobas_gazebo_common/constants.hpp>
#include <tobas_node/node.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_tools/control_latency_publisher.hpp>

#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_gazebo_msgs/msg/throttle.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>

namespace tobas
{
namespace gazebo
{
class ElectricRotorCommandHandlerNode : public BaseNode
{
  using self = ElectricRotorCommandHandlerNode;
  using super = BaseNode;

public:
  explicit ElectricRotorCommandHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ElectricPropulsionSystemConfig::ConstSharedPtr eprop_;
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;

  std::map<std::string, ros2::PublisherPtr<tobas_gazebo_msgs::msg::Throttle>> throttle_pubs_;
  ControlLatencyPublisher latency_pub_;

  ros2::SubscriberPtr<Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Battery> battery_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorSpeedArray> tar_speeds_sub_;

  void droneCb(const Drone::ConstSharedPtr& drone);
  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void targetSpeedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& tar_speeds);
};

ElectricRotorCommandHandlerNode::ElectricRotorCommandHandlerNode(const rclcpp::NodeOptions& options)
  : super("gazebo_electric_rotor_command_handler", nodeOptions_Default(options)), latency_pub_(this)
{
  drone_sub_ = createSubscriber(topic::kDrone, &self::droneCb, this, true, true);
}

void ElectricRotorCommandHandlerNode::droneCb(const Drone::ConstSharedPtr& drone)
{
  if (!drone->prop) {
    return;
  }

  if (drone->prop->type() != PropulsionSystem::kElectric) {
    return;
  }

  eprop_ = boost::polymorphic_pointer_downcast<ElectricPropulsionSystemConfig>(drone->prop);

  // Register publishers.
  throttle_pubs_.clear();
  for (const auto& [link_name, _] : eprop_->rotors) {
    const auto topic = path::join(gazebo::kRotorThrottleCmdTopicNS, link_name);
    throttle_pubs_[link_name] = createPublisher<tobas_gazebo_msgs::msg::Throttle>(topic);
  }

  // Register subscribers.
  battery_sub_ = createSubscriber(topic::kBattery, &self::batteryCb, this);
  tar_speeds_sub_ = createSubscriber(topic::kRotorSpeedsCmd, &self::targetSpeedsCb, this);
}

void ElectricRotorCommandHandlerNode::batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery)
{
  battery_ = battery;
}

void ElectricRotorCommandHandlerNode::targetSpeedsCb(const tobas_msgs::msg::RotorSpeedArray::ConstSharedPtr& tar_speeds)
{
  if (!eprop_) {
    TOBAS_WARN_THROTTLE(kTypicalWarnPeriod, "Drone message has not been received yet.");
    return;
  }
  if (!battery_) {
    TOBAS_WARN_THROTTLE(kTypicalWarnPeriod, "Battery message has not been received yet.");
    return;
  }

  for (const auto& speed : tar_speeds->speeds) {
    // Check link name.
    if (!throttle_pubs_.contains(speed.link_name)) {
      TOBAS_ERROR("Electric rotor '" + speed.link_name + "' does not exist.");
      continue;
    }

    // Create throttle message.
    auto throttle = std::make_unique<tobas_gazebo_msgs::msg::Throttle>();
    throttle->header = tar_speeds->header;
    throttle->data = eprop_->getRotor(speed.link_name)->throttleFromSpeed(speed.speed, battery_->voltage);

    // Publish throttle message.
    throttle_pubs_.at(speed.link_name)->publish(std::move(throttle));
  }

  // Publish control latency.
  latency_pub_.publish(tar_speeds->header.stamp);
}
}  // namespace gazebo
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::gazebo::ElectricRotorCommandHandlerNode)

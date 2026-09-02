// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_gazebo_common/constants.hpp>
#include <tobas_node/node.hpp>
#include <tobas_path_tools/join.hpp>

#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_gazebo_msgs/msg/joint_command.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>

namespace tobas
{
namespace gazebo
{
/**
 * @brief Receive joint position, velocity, and force commands and send them to Gazebo.
 */
class JointCommandHandlerNode : public BaseNode
{
  using self = JointCommandHandlerNode;
  using super = BaseNode;

public:
  explicit JointCommandHandlerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  Drone::ConstSharedPtr drone_;

  std::unordered_map<
    std::string,
    std::pair<JointCommandInterface, ros2::PublisherPtr<tobas_gazebo_msgs::msg::JointCommand>>>
    ctrl_map_;

  ros2::SubscriberPtr<Drone> drone_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointCommandArray> positions_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointCommandArray> velocities_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointCommandArray> efforts_sub_;

  void publishJointCommand(const std::string& jnt_name, double command);
  void publishJointCommand(const tobas_msgs::msg::JointCommand& cmd);

  void droneCb(const Drone::ConstSharedPtr& drone);
  void jointPositionsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& positions);
  void jointVelocitiesCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& velocities);
  void jointEffortsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& efforts);
};

JointCommandHandlerNode::JointCommandHandlerNode(const rclcpp::NodeOptions& options)
  : super("gazebo_joint_command_handler", nodeOptions_Default(options))
{
  drone_sub_ = createSubscriber(topic::kDrone, &self::droneCb, this);
}

void JointCommandHandlerNode::publishJointCommand(const std::string& jnt_name, double command)
{
  auto gz_cmd = std::make_unique<tobas_gazebo_msgs::msg::JointCommand>();
  gz_cmd->data = command;

  const auto& publisher = ctrl_map_.at(jnt_name).second;
  publisher->publish(std::move(gz_cmd));
}

void JointCommandHandlerNode::publishJointCommand(const tobas_msgs::msg::JointCommand& cmd)
{
  publishJointCommand(cmd.name, cmd.data);
}

void JointCommandHandlerNode::droneCb(const Drone::ConstSharedPtr& drone)
{
  drone_ = drone;

  // Resister publishers
  ctrl_map_.clear();
  for (const auto& [_, joint] : drone->joints) {
    const auto topic = path::join(gazebo::kJointCommandTopicNS, joint.name);
    ctrl_map_[joint.name] = { static_cast<JointCommandInterface>(joint.cmd_iface),
                              createPublisher<tobas_gazebo_msgs::msg::JointCommand>(topic, false, true) };
  }

  // Resister subscribers
  positions_sub_ = createSubscriber(topic::kJointPosCmd, &self::jointPositionsCmdCb, this);
  velocities_sub_ = createSubscriber(topic::kJointVelCmd, &self::jointVelocitiesCmdCb, this);
  efforts_sub_ = createSubscriber(topic::kJointEffCmd, &self::jointEffortsCmdCb, this);
}

void JointCommandHandlerNode::jointPositionsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& positions)
{
  for (const auto& tbs_cmd : positions->commands) {
    const auto& jnt_name = tbs_cmd.name;
    if (!ctrl_map_.contains(jnt_name)) {
      TOBAS_ERROR("Controller for joint '", jnt_name, "' is not found.");
      continue;
    }

    const auto& cmd_iface = ctrl_map_[jnt_name].first;
    if (cmd_iface != JointCommandInterface::kPosition) {
      TOBAS_ERROR(
        "The command interface of joint '",
        jnt_name,
        "' is not position. So received position command for joint '",
        jnt_name,
        "' is ignored.");
      continue;
    }

    publishJointCommand(tbs_cmd);
  }
}

void JointCommandHandlerNode::jointVelocitiesCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& velocities)
{
  for (const auto& tbs_cmd : velocities->commands) {
    const auto& jnt_name = tbs_cmd.name;

    if (!ctrl_map_.contains(jnt_name)) {
      TOBAS_ERROR("Controller for joint '", jnt_name, "' is not found.");
      continue;
    }

    const auto& cmd_iface = ctrl_map_[jnt_name].first;
    if (cmd_iface != JointCommandInterface::kVelocity) {
      TOBAS_ERROR(
        "The command interface of joint '",
        jnt_name,
        "' is not velocity. So received velocity command for joint '",
        jnt_name,
        "' is ignored.");
      continue;
    }

    publishJointCommand(tbs_cmd);
  }
}

void JointCommandHandlerNode::jointEffortsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& efforts)
{
  for (const auto& tbs_cmd : efforts->commands) {
    const auto& jnt_name = tbs_cmd.name;
    if (!ctrl_map_.contains(jnt_name)) {
      TOBAS_ERROR("Controller for joint '", jnt_name, "' is not found.");
      continue;
    }

    const auto& cmd_iface = ctrl_map_[jnt_name].first;
    if (cmd_iface != JointCommandInterface::kEffort) {
      TOBAS_ERROR(
        "The command interface of joint '",
        jnt_name,
        "' is not effort. So received effort command for joint '",
        jnt_name,
        "' is ignored.");
      continue;
    }

    publishJointCommand(tbs_cmd);
  }
}
}  // namespace gazebo
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::gazebo::JointCommandHandlerNode)

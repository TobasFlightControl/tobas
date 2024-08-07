#pragma once

#include <tobas_node/node.hpp>
#include <tobas_msgs/JointCommandArray.h>

namespace tobas_gazebo_ros
{
/**
 * @brief ジョイントの位置，速度，力のコマンドを受け取り，Gazeboのトランスミッションに指令する．
 */
class JointCommandHandler : public tobas::BaseNode
{
  using self = JointCommandHandler;
  using super = tobas::BaseNode;

public:
  explicit JointCommandHandler(
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

private:
  enum command_type_t : int
  {
    POSITION,
    VELOCITY,
    EFFORT,
  };

  std::unordered_map<std::string, std::pair<command_type_t, PublisherPtr<>>> ctrl_map_;
  SubscriberPtr<> positions_sub_;
  SubscriberPtr<> velocities_sub_;
  SubscriberPtr<> efforts_sub_;

  bool initialize();

  void jointPositionsCmdCb(const tobas_msgs::JointCommandArray::ConstSharedPtr& positions);
  void jointVelocitiesCmdCb(const tobas_msgs::JointCommandArray::ConstSharedPtr& velocities);
  void jointEffortsCmdCb(const tobas_msgs::JointCommandArray::ConstSharedPtr& efforts);
};
}  // namespace tobas_gazebo_ros

#pragma once

#include <tobas_node/node.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>

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
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

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

  void jointPositionsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& positions);
  void jointVelocitiesCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& velocities);
  void jointEffortsCmdCb(const tobas_msgs::msg::JointCommandArray::ConstSharedPtr& efforts);
};
}  // namespace tobas_gazebo_ros

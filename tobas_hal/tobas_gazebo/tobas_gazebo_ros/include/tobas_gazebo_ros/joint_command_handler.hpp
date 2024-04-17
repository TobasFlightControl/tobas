#pragma once

#include <tobas_tools/node.hpp>
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
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  enum command_type_t : int
  {
    POSITION,
    VELOCITY,
    EFFORT,
  };

  std::unordered_map<std::string, std::pair<command_type_t, ros::Publisher>> ctrl_map_;
  ros::Subscriber positions_sub_;
  ros::Subscriber velocities_sub_;
  ros::Subscriber efforts_sub_;

  int initialize();

  void jointPositionsCmdCb(const tobas_msgs::JointCommandArrayConstPtr& positions);
  void jointVelocitiesCmdCb(const tobas_msgs::JointCommandArrayConstPtr& velocities);
  void jointEffortsCmdCb(const tobas_msgs::JointCommandArrayConstPtr& efforts);
};
}  // namespace tobas_gazebo_ros

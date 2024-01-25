#pragma once

#include <tobas_tools/node.hpp>
#include <tobas_msgs/JointPositions.h>
#include <tobas_msgs/JointVelocities.h>
#include <tobas_msgs/JointEfforts.h>

namespace tobas_gazebo
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

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  int initialize();

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void jointPositionsCmdCb(const tobas_msgs::JointPositionsConstPtr& positions);
  void jointVelocitiesCmdCb(const tobas_msgs::JointVelocitiesConstPtr& velocities);
  void jointEffortsCmdCb(const tobas_msgs::JointEffortsConstPtr& efforts);
};
}  // namespace tobas_gazebo

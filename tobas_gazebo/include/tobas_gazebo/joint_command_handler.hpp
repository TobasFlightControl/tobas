#pragma once

#include <sensor_msgs/JointState.h>

#include <tobas_tools/node.hpp>

namespace tobas_gazebo
{
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
  ros::Subscriber js_sub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  int initialize();

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void jointStateCmdCb(const sensor_msgs::JointStateConstPtr& js);
};
}  // namespace tobas_gazebo

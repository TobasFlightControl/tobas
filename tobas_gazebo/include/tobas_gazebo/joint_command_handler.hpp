#pragma once

#include <sensor_msgs/JointState.h>

#include <tobas_tools/node.hpp>

namespace tobas_gazebo
{
struct JointControlInfo
{
  std::string controller_name;

  enum command_type_t : int
  {
    POSITION,
    VELOCITY,
    EFFORT,
  } command_type;
};

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
  std::unordered_map<std::string, JointControlInfo> ctrl_info_map_;

  // PubSub
  std::unordered_map<std::string, ros::Publisher> cmd_pub_map_;
  ros::Subscriber js_sub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  void getCommandTypes();

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void jointStateCmdCb(const sensor_msgs::JointStateConstPtr& js);
};
}  // namespace tobas_gazebo

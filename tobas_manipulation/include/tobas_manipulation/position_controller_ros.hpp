#pragma once

#include <sensor_msgs/JointState.h>

#include <tobas_kdl/treejointstateconverter.hpp>
#include <tobas_kdl/treeactivejointsextractor.hpp>
#include <tobas_ros_tools/tf_listener.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/JointCommandArray.h>
#include <tobas_msgs/LinkStateArray.h>

namespace tobas_manipulation
{
class PositionControllerRos : public tobas::BaseNode
{
  using self = PositionControllerRos;
  using super = tobas::BaseNode;

public:
  explicit PositionControllerRos(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  tobas::Drone drone_;

  sensor_msgs::JointState home_js_;
  ros::Time t_last_cmd_;
  bool is_commanded_ = false;

  sensor_msgs::JointStateConstPtr tar_js_;
  tobas_msgs::LinkStateArrayConstPtr tar_ls_;

  // Publishers
  ros::Publisher positions_pub_;

  // Subscribers
  ros::Subscriber cur_js_sub_;
  ros::Subscriber tar_js_sub_;
  ros::Subscriber tar_ls_sub_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  int jointSpaceControl(tobas_msgs::JointCommandArray& positions_msg);
  int taskSpaceControl(tobas_msgs::JointCommandArray& positions_msg);

  void currentJointStateCb(const sensor_msgs::JointStateConstPtr& cur_js);
  void targetJointStateCb(const sensor_msgs::JointStateConstPtr& tar_js);
  void targetLinkStateCb(const tobas_msgs::LinkStateArrayConstPtr& tar_ls);
};
}  // namespace tobas_manipulation

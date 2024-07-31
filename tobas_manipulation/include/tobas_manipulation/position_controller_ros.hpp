#pragma once

#include <sensor_msgs/msg/joint_state.hpp>

#include <tobas_kdl/treejointstateconverter.hpp>
#include <tobas_kdl/treeactivejointsextractor.hpp>
#include <tobas_ros2_tools/tf_listener.hpp>
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
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

private:
  tobas::Drone drone_;

  sensor_msgs::msg::JointState home_js_;
  rclcpp::Time t_last_cmd_;
  bool is_commanded_ = false;

  sensor_msgs::msg::JointStateConstPtr tar_js_;
  tobas_msgs::LinkStateArrayConstPtr tar_ls_;

  // Publishers
  rclcpp::Publisher positions_pub_;

  // Subscribers
  rclcpp::Subscriber cur_js_sub_;
  rclcpp::Subscriber tar_js_sub_;
  rclcpp::Subscriber tar_ls_sub_;

  int jointSpaceControl(tobas_msgs::JointCommandArray& positions_msg);
  int taskSpaceControl(tobas_msgs::JointCommandArray& positions_msg);

  void currentJointStateCb(const sensor_msgs::msg::JointStateConstPtr& cur_js);
  void targetJointStateCb(const sensor_msgs::msg::JointStateConstPtr& tar_js);
  void targetLinkStateCb(const tobas_msgs::LinkStateArrayConstPtr& tar_ls);
};
}  // namespace tobas_manipulation

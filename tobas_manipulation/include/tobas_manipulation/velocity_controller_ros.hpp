#pragma once


#include <sensor_msgs/msg/joint_state.hpp>

#include <tobas_kdl/treejointstateconverter.hpp>
#include <tobas_kdl/treeactivejointsextractor.hpp>
#include <tobas_kdl/treetaskspacevelctrl.hpp>
#include <tobas_ros2_tools/tf_listener.hpp>
#include <tobas_node/node.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_msgs/JointCommandArray.h>
#include <tobas_msgs/LinkStateArray.h>

#include <tobas_manipulation/VelocityControllerConfig.h>

namespace tobas_manipulation
{
class VelocityControllerRos : public tobas::BaseNode
{
  using self = VelocityControllerRos;
  using super = tobas::BaseNode;

  using ConfigType = tobas_manipulation::VelocityControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit VelocityControllerRos(
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

private:
  tobas::Drone drone_;
  kdl::TreeJointStateConverter cur_js_conv_;
  kdl::TreeJointStateConverter tar_js_conv_;
  kdl::TreeActiveJointsExtractor active_jnts_extractor_;
  kdl::TreeTaskSpaceVelCtrl vel_ctrl_;

  ros2::TransformListener tf_listener_;
  double jnt_time_const_;
  sensor_msgs::msg::JointState home_js_;
  rclcpp::Time t_last_cmd_;
  bool is_commanded_ = false;

  sensor_msgs::msg::JointState::ConstSharedPtr cur_js_;
  sensor_msgs::msg::JointState::ConstSharedPtr tar_js_;
  tobas_msgs::LinkStateArray::ConstSharedPtr tar_ls_;

  // Publishers
  PublisherPtr<> velocities_pub_;

  // Subscribers
  SubscriberPtr<> cur_js_sub_;
  SubscriberPtr<> tar_js_sub_;
  SubscriberPtr<> tar_ls_sub_;



  int jointSpaceControl(tobas_msgs::JointCommandArray& velocities_msg);
  int taskSpaceControl(tobas_msgs::JointCommandArray& velocities_msg);

  void currentJointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& cur_js);
  void targetJointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& tar_js);
  void targetLinkStateCb(const tobas_msgs::LinkStateArray::ConstSharedPtr& tar_ls);


};
}  // namespace tobas_manipulation

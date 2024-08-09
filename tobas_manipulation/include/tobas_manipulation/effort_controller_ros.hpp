#pragma once


#include <sensor_msgs/msg/joint_state.hpp>

#include <tobas_kdl/treejointstateconverter.hpp>
#include <tobas_kdl/treeactivejointsextractor.hpp>
#include <tobas_kdl/treejntspacepid.hpp>
#include <tobas_kdl/treetaskspacepid.hpp>
#include <tobas_ros2_tools/tf_listener.hpp>

#include <tobas_node/node.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_msgs/JointCommandArray.h>
#include <tobas_msgs/LinkStateArray.h>

#include <tobas_manipulation/EffortControllerConfig.h>

namespace tobas_manipulation
{
class EffortControllerRos : public tobas::BaseNode
{
  using self = EffortControllerRos;
  using super = tobas::BaseNode;

  using ConfigType = tobas_manipulation::EffortControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit EffortControllerRos(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone drone_;
  kdl::TreeJointStateConverter cur_js_conv_;
  kdl::TreeJointStateConverter tar_js_conv_;
  kdl::TreeActiveJointsExtractor active_jnts_extractor_;
  kdl::TreeJntSpacePID pid_js_;
  kdl::TreeTaskSpacePID pid_ts_;

  ros2::TransformListener tf_listener_;
  sensor_msgs::msg::JointState home_js_;
  rclcpp::Time t_last_cmd_;
  bool is_commanded_ = false;

  sensor_msgs::msg::JointState::ConstSharedPtr cur_js_;
  sensor_msgs::msg::JointState::ConstSharedPtr tar_js_;
  tobas_msgs::LinkStateArray::ConstSharedPtr tar_ls_;

  // Publishers
  PublisherPtr<> efforts_pub_;

  // Subscribers
  SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  SubscriberPtr<> cur_js_sub_;
  SubscriberPtr<> tar_js_sub_;
  SubscriberPtr<> tar_ls_sub_;



  int jointSpaceControl(tobas_msgs::JointCommandArray& efforts_msg);
  int taskSpaceControl(tobas_msgs::JointCommandArray& efforts_msg);

  void currentJointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& cur_js);
  void targetJointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& tar_js);
  void targetLinkStateCb(const tobas_msgs::LinkStateArray::ConstSharedPtr& tar_ls);


};
}  // namespace tobas_manipulation

#pragma once


#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tobas_kdl/jntarray.hpp>
#include <tobas_kdl/treejntparser.hpp>
#include <tobas_kdl/treejointstateconverter.hpp>
#include <tobas_node/node.hpp>
#include <tobas_tools/command_level_handler.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_pose_pid/position_pid.hpp>
#include <tobas_pose_pid/orientation_pid.hpp>
#include <tobas_msgs/Odometry.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/PoseTwistAccelCommand.hpp>

#include <tobas_np_pid/ControllerConfig.h>

#include "./mixer.hpp"

namespace tobas_np_pid
{
class ControllerNode : public tobas::BaseNode
{
  using self = ControllerNode;
  using super = tobas::BaseNode;

  using ConfigType = tobas_np_pid::ControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit ControllerNode(
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

private:
  // Drone
  tobas::Drone drone_;
  kdl::TreeJointStateConverter js_converter_;

  // Controllers
  tobas::PositionPid pos_pid_;
  tobas::OrientationPid ori_pid_;
  Mixer mixer_;

  // Dynamic parameters
  tobas::PositionPidConfig pos_cfg_;
  tobas::OrientationPidConfig ori_cfg_;
  MixerConfig mixer_cfg_;

  // Mutable variables
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;
  sensor_msgs::msg::JointState::ConstSharedPtr js_;
  std_msgs::msg::Bool::ConstSharedPtr arming_;
  tobas_msgs::PoseTwistAccelCommandPtr cmd_;
  tobas::CommandLevelHandler cmd_level_handler_;

  // Publishers
  PublisherPtr<tobas_msgs::msg::RotorSpeeds> rot_speeds_pub_;
  PublisherPtr<> feedback_pub_;

  // Subscribers
  SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  SubscriberPtr<tobas_msgs::msg::Battery> battery_sub_;
  SubscriberPtr<sensor_msgs::msg::JointState> js_sub_;
  SubscriberPtr<std_msgs::msg::Bool> arming_sub_;
  SubscriberPtr<> cmd_sub_;



  void registerPublishers();
  void registerSubscribers();
  bool isReadyToControl();

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void jointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& js);
  void armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming);
  void commandCb(const tobas_msgs::PoseTwistAccelCommand::ConstSharedPtr& cmd);


};
}  // namespace tobas_np_pid

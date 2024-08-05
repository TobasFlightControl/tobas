#pragma once

#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/Bool.h>

#include <tobas_kdl/jntarray.hpp>
#include <tobas_kdl/treejntparser.hpp>
#include <tobas_kdl/treejointstateconverter.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_tools/command_level_handler.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_tools/position_pid.hpp>
#include <tobas_tools/orientation_pid.hpp>
#include <tobas_msgs/Odometry.hpp>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/PoseTwistAccelCommand.hpp>

#include <tobas_np_pid/ControllerConfig.h>

#include "./mixer.hpp"

namespace tobas_np_pid
{
class ControllerRos : public tobas::BaseNode
{
  using self = ControllerRos;
  using super = tobas::BaseNode;

  using ConfigType = tobas_np_pid::ControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit ControllerRos(
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
  tobas_msgs::OdometryConstPtr odom_;
  tobas_msgs::BatteryConstPtr battery_;
  sensor_msgs::msg::JointStateConstPtr js_;
  std_msgs::BoolConstPtr arming_;
  tobas_msgs::PoseTwistAccelCommandPtr cmd_;
  tobas::CommandLevelHandler cmd_level_handler_;

  // Publishers
  rclcpp::Publisher rot_speeds_pub_;
  rclcpp::Publisher feedback_pub_;

  // Subscribers
  rclcpp::Subscriber odom_sub_;
  rclcpp::Subscriber battery_sub_;
  rclcpp::Subscriber js_sub_;
  rclcpp::Subscriber arming_sub_;
  rclcpp::Subscriber cmd_sub_;

  // Dynamic Reconfigure Server
  ConfigServer server_;

  void registerPublishers();
  void registerSubscribers();
  bool isReadyToControl();

  void odomCb(const tobas_msgs::OdometryConstPtr& odom);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void jointStateCb(const sensor_msgs::msg::JointStateConstPtr& js);
  void armingCb(const std_msgs::BoolConstPtr& arming);
  void commandCb(const tobas_msgs::PoseTwistAccelCommandConstPtr& cmd);

  void dynamicReconfigureCb(const ConfigType& cfg, size_t);
};
}  // namespace tobas_np_pid

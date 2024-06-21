#pragma once

#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

#include <tobas_kdl/jntarray.hpp>
#include <tobas_kdl/treejntparser.hpp>
#include <tobas_kdl/treejointstateconverter.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_tools/command_level_handler.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_tools/position_pid.hpp>
#include <tobas_tools/orientation_pid.hpp>
#include <tobas_msgs/Odometry.h>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/PoseTwistAccelCommand.h>

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
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

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
  sensor_msgs::JointStateConstPtr js_;
  std_msgs::BoolConstPtr arming_;
  tobas_msgs::PoseTwistAccelCommandPtr cmd_;
  tobas::CommandLevelHandler cmd_level_handler_;

  // Publishers
  ros::Publisher rot_speeds_pub_;
  ros::Publisher feedback_pub_;

  // Subscribers
  ros::Subscriber odom_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber js_sub_;
  ros::Subscriber arming_sub_;
  ros::Subscriber cmd_sub_;

  // Dynamic Reconfigure Server
  ConfigServer server_;

  void registerPublishers();
  void registerSubscribers();
  bool isReadyToControl();

  void odomCb(const tobas_msgs::OdometryConstPtr& odom);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void jointStateCb(const sensor_msgs::JointStateConstPtr& js);
  void armingCb(const std_msgs::BoolConstPtr& arming);
  void commandCb(const tobas_msgs::PoseTwistAccelCommandConstPtr& cmd);

  void dynamicReconfigureCb(const ConfigType& cfg, size_t);
};
}  // namespace tobas_np_pid

#pragma once

#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <tobas_kdl/treejointstateconverter.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_tools/command_level_handler.hpp>
#include <tobas_tools/position_pid.hpp>
#include <tobas_tools/orientation_pid.hpp>
#include <tobas_drone_tools/accel_attitude_converter.hpp>
#include <tobas_drone_tools/mixer.hpp>
#include <tobas_msgs/Odometry.hpp>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/PosVelAccYaw.hpp>
#include <tobas_msgs/RollPitchYawThrust.h>

#include <tobas_mr_pid/ControllerConfig.h>

namespace tobas_mr_pid
{
class ControllerRos : public tobas::BaseNode
{
  static constexpr bool kDefaultDoThrustCorrection = false;

  using self = ControllerRos;
  using super = tobas::BaseNode;

  using ConfigType = tobas_mr_pid::ControllerConfig;
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
  tobas::RotorAxisExtractor z_rotors_;

  // rosparams
  bool do_thrust_correction_;

  // Controllers
  tobas::PositionPid pos_ctrl_;
  tobas::AccelAttitudeConverter acc_ctrl_;
  tobas::OrientationPid ori_ctrl_;
  tobas::Mixer mixer_;

  // Dynamic parameters
  tobas::PositionPidConfig pos_cfg_;
  tobas::AccelAttitudeConverterConfig acc_cfg_;
  tobas::OrientationPidConfig ori_cfg_;

  // Mutable variables
  tobas_msgs::OdometryConstPtr odom_;
  tobas_msgs::BatteryConstPtr battery_;
  sensor_msgs::msg::JointStateConstPtr js_;
  std_msgs::Float64ConstPtr thrust_corr_factor_;
  std_msgs::BoolConstPtr arming_;
  tobas_msgs::PosVelAccYawPtr tar_pvay_W_;      // PosVelYawの目標値 (世界座標系)
  tobas_msgs::RollPitchYawThrustPtr tar_rpyt_;  // RollPitchYawThrustの目標値
  tobas::CommandLevelHandler cmd_level_handler_;

  // Publishers
  rclcpp::Publisher rot_speeds_pub_;
  rclcpp::Publisher feedback_pub_;

  // Subscribers
  rclcpp::Subscriber odom_sub_;
  rclcpp::Subscriber battery_sub_;
  rclcpp::Subscriber js_sub_;
  rclcpp::Subscriber thrust_corr_factor_sub_;
  rclcpp::Subscriber arming_sub_;
  rclcpp::Subscriber pvay_sub_;
  rclcpp::Subscriber rpyt_sub_;

  // Dynamic Reconfigure Server
  ConfigServer server_;

  void getRosParams();
  void registerPublishers();
  void registerSubscribers();
  bool isReadyToControl();

  void odomCb(const tobas_msgs::OdometryConstPtr& odom);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void jointStateCb(const sensor_msgs::msg::JointStateConstPtr& js);
  void thrustCorrectionFactorCb(const std_msgs::Float64ConstPtr& msg);
  void armingCb(const std_msgs::BoolConstPtr& arming);
  void posVelAccYawCb(const tobas_msgs::PosVelAccYawConstPtr& pvay);
  void rpyThrustCb(const tobas_msgs::RollPitchYawThrustConstPtr& rpyt);

  void dynamicReconfigureCb(const ConfigType& cfg, size_t);
};
}  // namespace tobas_mr_pid

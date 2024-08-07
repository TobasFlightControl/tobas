#pragma once

#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <tobas_kdl/treejointstateconverter.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_tools/command_level_handler.hpp>
#include <tobas_msgs/Odometry.hpp>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/Wind.h>
#include <tobas_msgs/RotorSpeeds.h>
#include <tobas_msgs/PosVelAccYaw.hpp>
#include <tobas_msgs/RollPitchYawThrust.h>
#include <tobas_msgs/RotorSpeeds.h>

#include <tobas_drone_tools/accel_attitude_converter.hpp>
#include <tobas_mr_mpc/ControllerConfig.h>

#include "./position_controller.hpp"
#include "./orientation_controller.hpp"

namespace tobas_mr_mpc
{
class ControllerRos : public tobas::BaseNode
{
  static constexpr bool kDefaultDoThrustCorrection = false;

  using self = ControllerRos;
  using super = tobas::BaseNode;

  using ConfigType = tobas_mr_mpc::ControllerConfig;
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
  tobas_mr_mpc::PositionController pos_ctrl_;
  tobas::AccelAttitudeConverter acc_ctrl_;
  tobas_mr_mpc::OrientationController ori_ctrl_;

  // Dynamic parameters
  tobas_mr_mpc::PositionControllerConfig pos_cfg_;
  tobas::AccelAttitudeConverterConfig acc_cfg_;
  tobas_mr_mpc::OrientationControllerConfig ori_cfg_;

  // Mutable variables
  tobas_msgs::OdometryConstPtr odom_;
  tobas_msgs::BatteryConstPtr battery_;
  tobas_msgs::WindConstPtr wind_;  // 風速 (世界座標系)
  tobas_msgs::RotorSpeedsConstPtr rotor_speeds_;
  sensor_msgs::msg::JointStateConstPtr js_;
  std_msgs::Float64ConstPtr thrust_corr_factor_;
  std_msgs::BoolConstPtr arming_;
  tobas_msgs::PosVelAccYawPtr tar_pvay_W_;      // PosVelYawの目標値 (世界座標系)
  tobas_msgs::RollPitchYawThrustPtr tar_rpyt_;  // RollPitchYawThrustの目標値
  kdl::Vector tar_acc_fb_;
  tobas::CommandLevelHandler cmd_level_handler_;

  // Publishers
  rclcpp::Publisher rot_speeds_pub_;
  rclcpp::Publisher feedback_pub_;

  // Subscribers
  rclcpp::Subscriber odom_sub_;
  rclcpp::Subscriber battery_sub_;
  rclcpp::Subscriber wind_sub_;
  rclcpp::Subscriber rotor_speeds_sub_;
  rclcpp::Subscriber arming_sub_;
  rclcpp::Subscriber js_sub_;
  rclcpp::Subscriber thrust_corr_factor_sub_;
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
  void windCb(const tobas_msgs::WindConstPtr& wind);
  void rotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& rotor_speeds);
  void jointStateCb(const sensor_msgs::msg::JointStateConstPtr& js);
  void thrustCorrectionFactorCb(const std_msgs::Float64ConstPtr& msg);
  void armingCb(const std_msgs::BoolConstPtr& arming);
  void posVelAccYawCb(const tobas_msgs::PosVelAccYawConstPtr& pvay);
  void rpyThrustCb(const tobas_msgs::RollPitchYawThrustConstPtr& rpyt);

  void dynamicReconfigureCb(const ConfigType& cfg, size_t);
};
}  // namespace tobas_mr_mpc

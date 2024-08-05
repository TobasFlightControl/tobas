#pragma once

#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/FluidPressure.h>
#include <std_msgs/Bool.h>

#include <tobas_linear_control/lqd.hpp>
#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_tools/rotor_axis_extractor.hpp>
#include <tobas_tools/micro_disturbance_eom.hpp>
#include <tobas_msgs/Odometry.hpp>
#include <tobas_msgs/msg/speed_roll_delta_pitch.hpp>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/ControlSurfaceDeflections.h>
#include <tobas_msgs/FixedWingControllerFeedback.h>
#include <tobas_fixed_wing_lqd/ControllerConfig.h>

namespace tobas_fixed_wing_lqd
{
class Controller : public tobas::BaseNode
{
  using self = Controller;
  using super = tobas::BaseNode;

  using ConfigType = tobas_fixed_wing_lqd::ControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit Controller(, const std::string& name = rclcpp::this_node::getName());

private:
  // Drone
  tobas::Drone drone_;
  tobas::RotorAxisExtractor x_rotors_;
  tobas::MicroDisturbanceEoM eom_;  // 微小擾乱状態方程式

  // 固定値
  kdl::JntArray q_0_;

  double cur_roll_, cur_pitch_, cur_yaw_;
  sensor_msgs::msg::FluidPressureConstPtr air_pressure_;  // 大気圧
  tobas_msgs::BatteryConstPtr battery_;              // 現在のバッテリーの状態
  tobas_msgs::OdometryConstPtr odom_nwu_;            // 現在の状態 (NWU座標系)
  tobas_msgs::SpeedRollDeltaPitchConstPtr cmd_nwu_;  // 現在のコマンド (NWU座標系)
  tobas_msgs::Odometry odom_ned_;                    // 現在の状態 (NED座標系)
  std_msgs::BoolConstPtr arming_;                    // ロータのアーム状態
  tobas_msgs::msg::SpeedRollDeltaPitch cmd_ned_;          // 現在のコマンド (NED座標系)

  ctrl::LQD lqd_;  // 最適レギュレータ

  // PubSub
  rclcpp::Publisher rot_speeds_pub_;
  rclcpp::Publisher deflections_pub_;
  rclcpp::Publisher feedback_pub_;
  rclcpp::Subscriber air_pressure_sub_;
  rclcpp::Subscriber battery_sub_;
  rclcpp::Subscriber odom_sub_;
  rclcpp::Subscriber arming_sub_;
  rclcpp::Subscriber cmd_sub_;

  // Dynamic Reconfigure
  ConfigServer server_;

  bool isReadyToControl();
  void setScales();
  void updateCurrentStateVector();
  void updateSetStateVector();
  void publishRotSpeeds(const Eigen::VectorXd& thrust);
  void publishDeflections(const Eigen::VectorXd& deflections);
  void publishFeedback(const Eigen::VectorXd& du);

  void airPressureCb(const sensor_msgs::msg::FluidPressureConstPtr& msg);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void odomCb(const tobas_msgs::OdometryConstPtr& odom_nwu);
  void armingCb(const std_msgs::BoolConstPtr& arming);
  void commandCb(const tobas_msgs::SpeedRollDeltaPitchConstPtr& cmd_nwu);

  void dynamicReconfigureCb(const ConfigType& cfg, size_t);
};
}  // namespace tobas_fixed_wing_lqd

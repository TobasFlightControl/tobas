#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/FluidPressure.h>

#include <tobas_ros_tools/timer.hpp>
#include <tobas_linear_control/c2d/rk4.hpp>
#include <tobas_linear_control/mpc/linear_dense.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_tools/rotor_axis_extractor.hpp>
#include <tobas_tools/micro_disturbance_eom.hpp>
#include <tobas_msgs/Odometry.h>
#include <tobas_msgs/SpeedRollDeltaPitch.h>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/ControlSurfaceDeflections.h>
#include <tobas_msgs/FixedWingControllerFeedback.h>
#include <tobas_fixed_wing_mpc/ControllerConfig.h>

namespace tobas_fixed_wing_mpc
{
class Controller : public tobas::BaseNode
{
  using super = tobas::BaseNode;

  using ConfigType = tobas_fixed_wing_mpc::ControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit Controller(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  tobas::Drone drone_;

  tobas::RotorAxisExtractor x_rotors_;
  tobas::MicroDisturbanceEoM eom_;  // 微小擾乱状態方程式

  // 固定値
  KDL::JntArray q_0_;

  bool is_initialized_ = false;
  double cur_roll_, cur_pitch_, cur_yaw_;
  sensor_msgs::FluidPressureConstPtr air_pressure_;  // 大気圧
  tobas_msgs::BatteryConstPtr battery_;              // 現在のバッテリーの状態
  tobas_msgs::OdometryConstPtr odom_nwu_;            // 現在の状態 (NWU座標系)
  tobas_msgs::SpeedRollDeltaPitchConstPtr cmd_nwu_;  // 現在のコマンド (NWU座標系)
  tobas_msgs::Odometry odom_ned_;                    // 現在の状態 (NED座標系)
  tobas_msgs::SpeedRollDeltaPitch cmd_ned_;          // 現在のコマンド (NED座標系)

  ctrl::C2D_RK4 c2d_;         // 状態方程式を離散化
  ctrl::LinearDenseMPC mpc_;  // 線形モデル予測制御

  // PubSub
  ros::Publisher rot_speeds_pub_;
  ros::Publisher deflections_pub_;
  ros::Publisher feedback_pub_;
  ros::Subscriber air_pressure_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber cmd_sub_;

  // Timer
  tobas_ros::Timer check_topics_timer_;

  // Dynamic Reconfigure
  ConfigServer server_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool isReady();
  void initialize();
  void setCz();
  void setScales();
  void setInputConstraint();
  void setInputRateConstraint();
  void updateCurrentStateVector();
  void updateSetStateVector();
  void publishRotSpeeds(const Eigen::VectorXd& thrust);
  void publishDeflections(const Eigen::VectorXd& deflections);
  void publishFeedback(const Eigen::VectorXd& du);

  void airPressureCb(const sensor_msgs::FluidPressureConstPtr& msg);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void odomCb(const tobas_msgs::OdometryConstPtr& odom_nwu);
  void commandCb(const tobas_msgs::SpeedRollDeltaPitchConstPtr& cmd_nwu);

  void checkTopicsTimerCb(const ros::TimerEvent&);
  void dynamicReconfigureCb(const ConfigType& cfg, size_t);
};
}  // namespace tobas_fixed_wing_mpc

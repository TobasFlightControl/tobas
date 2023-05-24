#pragma once

#include <memory>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/FluidPressure.h>

#include <dh_ros_tools/node.hpp>
#include <dh_linear_control/lqr.hpp>

#include <tobas_tools/drone.hpp>
#include <tobas_tools/rotor_axis_extractor.hpp>
#include <tobas_tools/micro_disturbance_eom.hpp>
#include <tobas_msgs/BaseState.h>
#include <tobas_msgs/SpeedRollDeltaPitch.h>
#include <tobas_msgs/RotorSpeeds.h>
#include <tobas_msgs/ControlSurfaceDeflections.h>
#include <tobas_msgs/FixedWingControllerFeedback.h>
#include <tobas_fixed_wing_lqr/ControllerConfig.h>

namespace tobas_fixed_wing_lqr
{
class Controller : public dh_ros::BaseNode
{
  using super = dh_ros::BaseNode;

  using StateMsg = tobas_msgs::BaseState;
  using CmdMsg = tobas_msgs::SpeedRollDeltaPitch;

  using ConfigType = tobas_fixed_wing_lqr::ControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit Controller();

private:
  enum State
  {
    START,
    TAKEOFF,
    FLIGHT,
    LANDING
  };

  tobas::Drone drone_;

  tobas::RotorAxisExtractor x_rotors_;
  tobas::MicroDisturbanceEoM eom_;  // 微小擾乱状態方程式

  // RosParams
  ConfigType cfg_;  // 動的パラメータの初期値

  // 固定値
  KDL::JntArray q_0_;

  bool pressure_received_;
  bool bs_received_;
  State state_;
  double air_density_;  // 現在の大気密度
  StateMsg bs_ned_;     // 現在の状態 (NED座標系)
  CmdMsg cmd_ned_;      // 現在のコマンド (NED座標系)
  tobas_msgs::RotorSpeeds rotor_speeds_msg_;
  tobas_msgs::ControlSurfaceDeflections deflections_msg_;
  tobas_msgs::FixedWingControllerFeedback feedback_msg_;

  ctrl::LQR lqr_;  // 最適レギュレータ

  // PubSub
  ros::Publisher rotor_speeds_pub_;
  ros::Publisher deflections_pub_;
  ros::Publisher feedback_pub_;
  ros::Subscriber air_pressure_sub_;
  ros::Subscriber base_state_sub_;
  ros::Subscriber cmd_sub_;

  // Dynamic Reconfigure
  ConfigServer server_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;
  void createTimers() override;

  bool isReady();
  void publishTakeoffCommand();
  void setInitialTarget();
  void runOnce();
  void setScales();
  void updateCurrentStateVector();
  void updateSetStateVector(double tar_roll, double tar_delta_pitch);
  void updateRotorSpeeds(const Eigen::VectorXd& thrust);
  void updateDeflections(const Eigen::VectorXd& deflections);
  void publishFeedback(const Eigen::VectorXd& du);
  void reconfigure(const ConfigType& cfg);

  void airPressureCb(const sensor_msgs::FluidPressure& msg);
  void baseStateCb(const StateMsg& bs_nwu);
  void commandCb(const CmdMsg& cmd_nwu);
  void checkTopicsTimerCb(const ros::TimerEvent& event) override;
  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t level);
};
}  // namespace tobas_fixed_wing_lqr

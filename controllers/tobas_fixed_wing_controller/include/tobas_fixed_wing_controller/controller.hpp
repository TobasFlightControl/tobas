#pragma once

#include <memory>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <dh_ros_tools/node.hpp>
#include <dh_linear_control/c2d/rk4.hpp>
#include <dh_linear_control/mpc/linear_dense.hpp>

#include <tobas_tools/drone.hpp>
#include <tobas_msgs/BaseState.h>
#include <tobas_msgs/SpeedRollDeltaPitch.h>
#include <tobas_msgs/RotorSpeeds.h>
#include <tobas_msgs/ControlSurfaceDeflections.h>
#include <tobas_fixed_wing_controller/ControllerConfig.h>

#include "./dynamics.hpp"

namespace tobas_fixed_wing_controller
{
class Controller : public dh_ros::BaseNode
{
  using super = dh_ros::BaseNode;

  using StateMsg = tobas_msgs::BaseState;
  using CmdMsg = tobas_msgs::SpeedRollDeltaPitch;

  using ConfigType = tobas_fixed_wing_controller::ControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit Controller();

private:
  Drone drone_;

  // RosParams
  ConfigType init_dynamic_config_;  // 動的パラメータの初期値

  // 固定値
  std::vector<uint32_t> hor_prop_idxes_;  // 使用するロータ (+X) の添字
  uint32_t num_hor_props_;                // 推力発生用プロペラの個数
  uint32_t num_cs_;                       // 制御面の個数
  uint32_t u_dim_;                        // MPCの制御変数の次元

  bool is_initialized_;
  tobas_msgs::BaseState cur_bs_;  // 現在の状態 (NUD座標系)
  tobas_msgs::RotorSpeeds rotor_speeds_msg_;
  tobas_msgs::ControlSurfaceDeflections deflections_msg_;

  std::shared_ptr<FixedWingMicroDisturbanceDynamics> cont_;  // 微小擾乱状態方程式
  std::shared_ptr<ctrl::C2D_RK4> c2d_;                       // 状態方程式を離散化
  ctrl::LinearDenseMPC mpc_;                                 // 線形モデル予測制御

  // PubSub
  ros::Publisher rotor_speeds_pub_;
  ros::Publisher deflections_pub_;
  ros::Subscriber base_state_sub_;
  ros::Subscriber cmd_sub_;

  // Dynamic Reconfigure
  ConfigServer server_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;
  void createTimers() override;

  void initialize();
  void runOnce();
  void setCz();
  void setInputConstraint();
  void setInputRateConstraint();
  void updateWeight_Q(double beta_weight, double rot_weight);
  void updateWeight_S(int thrust_weight_exp, int deflection_weight_exp);
  void updateWeight_R(int thrust_rate_weight_exp, int deflection_rate_weight_exp, double dt);
  void updateCurrentStateVector();
  void updateSetStateVector(double tar_roll, double tar_delta_pitch);
  void updateRotorSpeeds(const Eigen::VectorXd& thrust);
  void updateDeflections(const Eigen::VectorXd& deflections);

  void baseStateCb(const StateMsg& bs_nwu);
  void commandCb(const CmdMsg& cmd_nwu);
  void checkTopicsTimerCb(const ros::TimerEvent& event) override;
  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t level);
};
}  // namespace tobas_fixed_wing_controller

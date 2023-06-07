#pragma once

#include <memory>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/JointState.h>

#include <dh_kdl/euler.hpp>
#include <dh_kdl/treejntnameparser.hpp>
#include <dh_ros_tools/timer.hpp>
#include <dh_ros_tools/node.hpp>

#include <tobas_tools/drone.hpp>
#include <tobas_tools/rotor_axis_extractor.hpp>
#include <tobas_msgs/BaseState.h>
#include <tobas_msgs/VelocityYaw.h>
#include <tobas_msgs/RotorSpeeds.h>
#include <tobas_msgs/Battery.h>
#include <tobas_multirotor_controller/ControllerConfig.h>

#include "./velocity_controller.hpp"
#include "./acceleration_controller.hpp"
#include "./rotation_controller.hpp"

namespace tobas_multirotor_controller
{
/**
 * @brief 加速度制御器(解析計算)，姿勢制御器(MPC)を組み合わせた制御器．
 * vx, vy, vz, yaw_rateの目標値に追従する．
 */
class VelocityControllerRos : public dh_ros::BaseNode
{
  using super = dh_ros::BaseNode;

  using StateMsg = tobas_msgs::BaseState;
  using CmdMsg = tobas_msgs::VelocityYaw;

  using ConfigType = tobas_multirotor_controller::ControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit VelocityControllerRos();

private:
  tobas::Drone drone_;

  KDL::TreeJointNameParser jnt_name_parser_;
  tobas::RotorAxisExtractor z_rotors_;

  tobas_msgs::BaseState cur_bs_;  // 現在のベースの状態
  tobas_msgs::Battery battery_;   // 現在のバッテリーの状態
  KDL::JntArray q_;               // 全ての非固定関節の角度
  KDL::Vector tar_vel_W_;
  KDL::Vector tar_acc_W_;
  KDL::Euler tar_rpy_;
  double U_;
  Eigen::VectorXd u_opt_;

  bool is_transformable_;  // プロペラ以外の可動関節を持つか否か
  bool is_initialized_;
  bool bs_received_;
  bool battery_received_;
  bool js_received_;
  bool cmd_received_;
  tobas_msgs::RotorSpeeds rotor_speeds_;

  std::shared_ptr<VelocityController> vel_controller_;
  std::shared_ptr<AccelerationController> acc_controller_;
  std::shared_ptr<RotationController> rot_controller_;

  // RosParams
  double gravity_;
  VelocityControllerDynamicParams dynamic_params_vel_;
  RotationControllerDynamicParams dynamic_params_rot_;

  // Timer
  dh_ros::Timer check_topics_timer_;

  // PubSub
  ros::Publisher rotor_speeds_pub_;
  ros::Subscriber base_state_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber cmd_sub_;

  // Dynamic Reconfigure
  ConfigServer server_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool isReady();
  void initialize();
  void updateDynamicParams(const ConfigType& cfg);
  void runOnce();
  void ctrlInputToRotorSpeeds(const Eigen::VectorXd& u, tobas_msgs::RotorSpeeds& speeds);
  double maxU();

  void baseStateCb(const StateMsg& bs);
  void batteryCb(const tobas_msgs::Battery& battery);
  void jointStateCb(const sensor_msgs::JointState& js);
  void commandCb(const CmdMsg& cmd);

  void checkTopicsTimerCb(const ros::TimerEvent&);
  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t level);
};
}  // namespace tobas_multirotor_controller

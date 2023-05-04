#pragma once

#include <memory>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/JointState.h>

#include <dh_kdl/treejntnameparser.hpp>
#include <dh_ros_tools/timer.hpp>

#include <tobas_tools/rotor_property.hpp>
#include <tobas_msgs/PoseVelStamped.h>
#include <tobas_msgs/VelocityYaw.h>
#include <tobas_msgs/RotorSpeeds.h>
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
class VelocityControllerRos
{
  using StateMsg = tobas_msgs::PoseVelStamped;
  using CmdMsg = tobas_msgs::VelocityYaw;

  using ConfigType = tobas_multirotor_controller::ControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit VelocityControllerRos();

private:
  ros::NodeHandle nh_;

  KDL::Tree tree_;
  KDL::TreeJointNameParser jnt_name_parser_;

  // rosparams
  std::string drone_name_;
  std::string description_;
  uint32_t num_rotors_;
  std::vector<std::string> required_joints_;  // プロペラ以外の可動関節の名前のリスト
  double gravity_;
  double battery_voltage_;
  RotorConfigs rotor_configs_;
  VelocityControllerDynamicParams dynamic_params_vel_;
  RotationControllerDynamicParams dynamic_params_rot_;

  KDL::JntArray q_;  // 全ての非固定関節の角度
  Eigen::Vector3d cur_vel_W_;
  Eigen::Vector3d cur_rpy_;
  Eigen::Vector3d cur_angvel_B_;
  Eigen::Vector3d tar_vel_W_;
  Eigen::Vector3d tar_acc_W_;
  Eigen::Vector3d tar_rpy_;
  double U_;
  Eigen::VectorXd u_opt_;

  bool is_transformable_;  // プロペラ以外の可動関節を持つか否か
  bool is_initialized_;
  bool bs_received_;
  bool js_received_;
  bool cmd_received_;
  ros::Time t_last_;  // 最後に動作した時刻
  tobas_msgs::RotorSpeeds rotor_speeds_;

  std::shared_ptr<VelocityController> vel_controller_;
  std::shared_ptr<AccelerationController> acc_controller_;
  std::shared_ptr<RotationController> rot_controller_;

  // PubSub
  ros::Publisher rotor_speeds_pub_;
  ros::Subscriber base_state_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber cmd_sub_;

  ConfigServer server_;               // Dynamic Reconfigure
  dh_ros::Timer check_topics_timer_;  // Check if messages are received or not.

  void getRosParams();
  void registerPublishers();
  void registerSubscribers();
  bool isReady();
  void initialize(const tobas_msgs::PoseVel& bs);
  void updateDynamicParams(const ConfigType& cfg);
  void runOnce(const tobas_msgs::PoseVel& bs);
  void ctrlInputToRotorSpeeds(const Eigen::VectorXd& u, tobas_msgs::RotorSpeeds& speeds);

  void baseStateCb(const StateMsg& bs);
  void jointStateCb(const sensor_msgs::JointState& js);
  void commandCb(const CmdMsg& cmd);

  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t level);
  void checkTopicsTimerCb(const ros::TimerEvent&);
};
}  // namespace tobas_multirotor_controller

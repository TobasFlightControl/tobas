#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/JointState.h>

#include <dh_ros_tools/timer.hpp>

#include <tobas_tools/rotor_property.hpp>
#include <tobas_msgs/PoseVelStamped.h>
#include <tobas_msgs/RotorSpeeds.h>
#include <tobas_msgs/Command.h>
#include <tobas_msgs/ControllerFeedback.h>
#include <tobas_multirotor_controller/ControllerConfig.h>

#include "./position_controller.hpp"
#include "./acceleration_controller.hpp"
#include "./rotation_controller.hpp"

/**
 * @brief 位置制御器(PD制御)，加速度制御器(解析計算)，姿勢制御器(MPC)を組み合わせた制御器．
 * x, y, z, yawの目標値に追従する．
 */
class Controller
{
  using StateMsg = tobas_msgs::PoseVelStamped;
  using CmdMsg = tobas_msgs::Command;

  using ConfigType = tobas_multirotor_controller::ControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit Controller();
  ~Controller();

private:
  ros::NodeHandle nh_;

  KDL::Tree tree_;
  KDL::TreeKDLModel kdl_model_;

  // rosparams
  std::string drone_name_;
  std::string description_;
  uint32_t num_rotors_;
  std::vector<std::string> required_joints_;  // プロペラ以外の可動関節の名前のリスト
  RotorConfigs rotor_configs_;

  KDL::JntArray q_;                 // 全ての非固定関節の角度
  geometry_msgs::Vector3 pos_des_;  // {world}で表された目標位置
  double yaw_des_;                  // {world}で表されたヨー角の目標値
  CmdMsg cmd_;
  bool is_transformable_;           // プロペラ以外の可動関節を持つか否か
  bool is_initialized_;
  bool bs_received_;
  bool js_received_;
  bool cmd_received_;
  ros::Time t_last_;  // 最後に動作した時刻

  PositionController pos_controller_;
  AccelerationController acc_controller_;
  RotationController rot_controller_;
  tobas_msgs::ControllerFeedback feedback_;
  tobas_msgs::RotorSpeeds rotor_speeds_;

  // PubSub
  ros::Publisher rotor_speeds_pub_;
  ros::Publisher feedback_pub_;
  ros::Subscriber bs_sub_;
  ros::Subscriber js_sub_;
  ros::Subscriber cmd_sub_;

  ConfigServer server_;               // Dynamic Reconfigure
  dh_ros::Timer check_topics_timer_;  // Check if messages are received or not.

  void getRosParams();
  void registerPublishers();
  void registerSubscribers();
  bool isReady();
  void initialize();
  void runOnce(const tobas_msgs::PoseVel& bs);
  void updateDesiredState(const tobas_msgs::PoseVel& bs, double dt);
  void ctrlInputToRotorSpeeds(const std::vector<double>& u, tobas_msgs::RotorSpeeds& speeds);

  void bsCb(const StateMsg& bs);
  void jsCb(const sensor_msgs::JointState& js);
  void commandCb(const CmdMsg& cmd);

  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t level);
  void checkTopicsTimerCb(const ros::TimerEvent&);
};

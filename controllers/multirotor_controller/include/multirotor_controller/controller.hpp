#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/JointState.h>

#include "./position_controller.hpp"
#include "./acceleration_controller.hpp"
#include "./rotation_controller.hpp"

#include <multirotor_tools/rotor_property.hpp>
#include <multirotor_msgs/PoseVelStamped.h>
#include <multirotor_msgs/RotorSpeeds.h>
#include <multirotor_msgs/Command.h>
#include <multirotor_msgs/ControllerFeedback.h>
#include <multirotor_controller/ControllerConfig.h>

/**
 * @brief 位置制御器(PD制御)，加速度制御器(解析計算)，姿勢制御器(MPC)を組み合わせた制御器．
 * x, y, z, yawの目標値に追従する．
 */
class Controller
{
  using ConfigType = multirotor_controller::ControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit Controller();

private:
  ros::NodeHandle nh_;

  KDL::Tree tree_;
  KDL::TreeKDLModel kdl_model_;

  const uint32_t num_rotors_;
  const std::vector<std::string> required_joints_;  // プロペラ以外の可動関節の名前のリスト
  const bool transformable_;  // プロペラ以外の可動関節を持つか否か
  const RotorProperties rotor_props_;

  multirotor_msgs::PoseVel bs_;
  KDL::JntArray q_;
  multirotor_msgs::Command cmd_;
  bool js_subscribed_;
  bool cmd_subscribed_;

  PositionController pos_controller_;
  AccelerationController acc_controller_;
  RotationController rot_controller_;
  multirotor_msgs::ControllerFeedback feedback_;
  multirotor_msgs::RotorSpeeds rotor_speeds_;
  dh_ros::Stopwatch stopwatch_;

  // PubSub
  ros::Publisher rotor_speeds_pub_;
  ros::Publisher feedback_pub_;
  ros::Subscriber bs_sub_;
  ros::Subscriber js_sub_;
  ros::Subscriber cmd_sub_;

  // Dynamic Reconfigure
  ConfigServer server_;

  void runOnce();
  void
  rotorVelsFromCtrlInput(const std::vector<double>& u, multirotor_msgs::RotorSpeeds& rotor_speeds);
  bool allMsgReceived();

  void bsCb(const multirotor_msgs::PoseVelStamped& msg);
  void jsCb(const sensor_msgs::JointState& msg);
  void commandCb(const multirotor_msgs::Command& msg);

  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t level);
};

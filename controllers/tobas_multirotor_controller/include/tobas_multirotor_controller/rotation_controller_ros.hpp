#pragma once

#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/JointState.h>

#include <dh_kdl/treejntnameparser.hpp>
#include <dh_ros_tools/timer.hpp>
#include <dh_ros_tools/stopwatch.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_tools/drone.hpp>
#include <tobas_msgs/Event.h>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/BaseState.h>
#include <tobas_msgs/RollPitchYawThrust.h>
#include <tobas_msgs/RollPitchYawrateThrust.h>
#include <tobas_msgs/RotorSpeeds.h>
#include <tobas_multirotor_controller/ControllerConfig.h>

#include "./rotation_controller.hpp"

namespace tobas_multirotor_controller
{
/**
 * @brief 姿勢制御器(MPC)．
 * roll, pitch, yaw, thrustで指令する．
 */
class RotationControllerRos : public tobas::BaseNode
{
  using super = tobas::BaseNode;

  using ConfigType = tobas_multirotor_controller::ControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit RotationControllerRos(
    ros::NodeHandle nh,
    ros::NodeHandle pnh,
    std::string name = ros::this_node::getName());

private:
  tobas::Drone drone_;
  KDL::TreeJointNameParser jnt_name_parser_;
  tobas::RotorAxisExtractor z_rotors_;

  RotationController rot_controller_;

  bool is_transformable_;  // プロペラ以外の可動関節を持つか否か
  bool is_initialized_;
  bool battery_received_;
  bool bs_received_;
  bool js_received_;
  bool rpy_thrust_received_;
  bool rpyd_thrust_received_;
  ros::Time t_last_rpyd_thrust_;
  tobas_msgs::BatteryConstPtr battery_;        // 現在のバッテリーの状態
  tobas_msgs::BaseStateConstPtr bs_;           // 現在のベースの状態
  KDL::JntArray q_;                            // 全ての非固定関節の角度
  tobas_msgs::RollPitchYawThrust rpy_thrust_;  // 姿勢+推力 (入力)
  Eigen::VectorXd u_opt_;

  // RosParams
  RotationControllerDynamicParams dynamic_params_;

  // Timer
  dh_ros::Timer check_topics_timer_;

  // PubSub
  ros::Publisher rotor_speeds_pub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber base_state_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber rpy_thrust_sub_;
  ros::Subscriber rpyd_thrust_sub_;

  // Dynamic Reconfigure
  ConfigServer server_;

  // Other
  dh_ros::Stopwatch stopwatch_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool isReady();
  void initialize();
  void updateDynamicParams(const ConfigType& cfg);
  void runOnce();
  double maxThrustSum();
  double minThrustSum();
  bool isCommandLevelOk(const tobas_msgs::CommandLevel& level);
  void updateTargetRoll(double tar_roll);
  void updateTargetPitch(double tar_pitch);
  void updateTargetThrust(double tar_thrust);

  void eventCb(const tobas_msgs::EventConstPtr& event) override;
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void baseStateCb(const tobas_msgs::BaseStateConstPtr& bs);
  void jointStateCb(const sensor_msgs::JointStateConstPtr& js);
  void rpyThrustCb(const tobas_msgs::RollPitchYawThrustConstPtr& rpy_thrust);
  void rpydThrustCb(const tobas_msgs::RollPitchYawrateThrustConstPtr& rpyd_thrust);

  void checkTopicsTimerCb(const ros::TimerEvent&);
  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t);
};
}  // namespace tobas_multirotor_controller

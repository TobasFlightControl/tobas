#pragma once

#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Config.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/ParamSet.h>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/Odometry.h>

#include <tobas_mr_arducopter/ControllerConfig.h>

#include "./socket.hpp"

namespace tobas_mr_arducopter
{
/**
 * @brief ArduCopter Controller \n
 * cf. [ArduPilot Gazebo Plugin](https://github.com/ArduPilot/ardupilot_gazebo)
 */
class ParamServerRos : public tobas::BaseNode
{
  static constexpr char kParamSetSrv[] = "mavros/param/set";
  static constexpr double kActivationDelayFromFirstPose = 5.;  // [s]

  using self = ParamServerRos;
  using super = tobas::BaseNode;

  using ConfigType = tobas_mr_arducopter::ControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit ParamServerRos(
    const ros::NodeHandle& nh,
    const ros::NodeHandle& pnh,
    const std::string& name = ros::this_node::getName());

private:
  // rosparams
  int frame_class_;
  int frame_type_;

  bool is_first_update_ = true;
  bool is_init_params_set_ = false;
  dynamic_reconfigure::ConfigConstPtr init_cfg_;
  std::unordered_map<std::string, int> ints_;        // Int parameters
  std::unordered_map<std::string, double> doubles_;  // Double parameters

  mavros_msgs::ParamSet param_set_msg_;
  ros::ServiceClient param_set_sc_;

  ros::Publisher server_state_pub_;
  ros::Subscriber state_sub_;
  ros::Subscriber local_pos_sub_;
  ros::Subscriber param_updates_sub_;

  ros::Timer config_timer_;
  ros::Timer set_init_config_timer_;
  ros::Timer set_init_params_timer_;

  ConfigServer server_;

  void getRosParams();
  void setParams(const dynamic_reconfigure::ConfigConstPtr& cfg);

  void stateCb(const mavros_msgs::StateConstPtr& state);
  void localPositionCb(const geometry_msgs::PoseStampedConstPtr&);
  void paramUpdatesCb(const dynamic_reconfigure::ConfigConstPtr& cfg);

  void setInitConfigTimerCb(const ros::TimerEvent&);
  void setInitParamsTimerCb(const ros::TimerEvent&);
};
}  // namespace tobas_mr_arducopter

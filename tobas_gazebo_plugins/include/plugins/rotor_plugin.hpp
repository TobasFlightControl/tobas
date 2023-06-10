#pragma once

#include <random>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <tobas_msgs/RotorSpeeds.h>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/WindSpeed.h>
#include <tobas_msgs/RotorDebug.h>

#include "../../include/tobas_gazebo_plugins/first_order_filter.hpp"

namespace gazebo
{
// Constants
static const std::string kPluginName = "motor_model_plugin";

// Default values
static const std::string kDefaultDebugPubTopic = "ground_truth/rotor_debug";
static const std::string kDefaultCmdSubTopic = "command/motor_speed";

class GazeboRotorPlugin : public ModelPlugin
{
  using super = ModelPlugin;

public:
  explicit GazeboRotorPlugin();

protected:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  ros::NodeHandle nh_;

  // SDF parameters
  std::string ns_;
  std::string link_name_;
  std::string joint_name_;
  int motor_number_;
  int direction_;  // turning direction. 1(CCW) or -1(CW).
  std::string debug_pub_topic_;
  std::string cmd_sub_topic_;
  std::string battery_sub_topic_;
  std::string wind_speed_sub_topic_;
  double kv_;  // Kv (with efficiency)
  double motor_const_;
  double moment_const_;
  double rotor_drag_coef_;
  double time_const_up_;
  double time_const_down_;
  double rotor_speed_slowdown_sim_;
  double check_delay_threshold_;
  double auto_reset_time_thr_;

  double cmd_rot_speed_;                   // [rad/s]
  tobas_msgs::Battery battery_;
  ignition::math::Vector3d wind_speed_W_;  // [m/s]
  double prev_sim_time_;                   // [s]
  double last_cmd_time_;                   // [s]
  bool is_activated_;
  bool is_initialized_;
  bool battery_received_;
  bool wind_speed_received_;
  FirstOrderFilter<double> rotor_speed_filter_;
  tobas_msgs::RotorDebug debug_msg_;

  physics::ModelPtr model_;
  physics::JointPtr joint_;
  physics::LinkPtr link_;
  physics::LinkPtr parent_link_;
  event::ConnectionPtr update_connection_;

  // PubSub
  ros::Publisher debug_pub_;
  ros::Subscriber command_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber wind_speed_sub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate(const common::UpdateInfo& info);
  void registerPubSub();
  bool isReady();
  void applyForceAndTorque(double rot_speed, const common::Time cur_time);
  void updateRotationSpeed(double dt);

  void commandCb(const tobas_msgs::RotorSpeeds& cmd);
  void batteryCb(const tobas_msgs::Battery& battery);
  void windSpeedCb(const tobas_msgs::WindSpeed& wind);
};
}  // namespace gazebo

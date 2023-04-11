#pragma once

#include <random>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <std_msgs/Float64.h>

#include <multirotor_msgs/RotorSpeeds.h>
#include <multirotor_msgs/WindSpeed.h>

#include "../../include/multirotor_gazebo_plugins/utils.hpp"
#include "../../include/multirotor_gazebo_plugins/first_order_filter.hpp"

namespace gazebo
{
// Constants
static constexpr char kPluginName[] = "motor_model_plugin";

// Default values
static constexpr char kDefaultSpeedPubTopic[] = "motor_speed";
static constexpr char kDefaultCmdSubTopic[] = "command/motor_speed";
static constexpr char kDefaultWindSubTopic[] = "wind_speed";
static constexpr double kDefaultRotorSpeedSlowdownSim = 10.;

class GazeboRotorPlugin : public ModelPlugin
{
  using CmdMsg = multirotor_msgs::RotorSpeeds;
  using WindMsg = multirotor_msgs::WindSpeed;

public:
  GazeboRotorPlugin();

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
  std::string motor_speed_pub_topic_;
  std::string cmd_sub_topic_;
  std::string wind_speed_sub_topic_;
  double max_rot_vel_;
  double motor_const_;
  double moment_const_;
  double rotor_drag_coef_;
  double roll_moment_coef_;
  double time_const_up_;
  double time_const_down_;
  double rotor_speed_slowdown_sim_;

  double ref_motor_input_;
  double prev_sim_time_;
  std_msgs::Float64 motor_speed_msg_;
  ignition::math::Vector3d wind_speed_W_;
  FirstOrderFilter<double> rotor_speed_filter_;

  physics::ModelPtr model_;
  physics::JointPtr joint_;
  physics::LinkPtr link_;
  physics::LinkPtr parent_link_;
  event::ConnectionPtr update_connection_;

  ros::Publisher motor_speed_pub_;
  ros::Subscriber command_sub_;
  ros::Subscriber wind_speed_sub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate(const common::UpdateInfo& info);
  void updateForcesAndMoments(double dt);

  void commandCb(const CmdMsg& cmd);
  void windSpeedCb(const WindMsg& wind);
};
}  // namespace gazebo

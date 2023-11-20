#pragma once

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <tobas_msgs/RotorSpeeds.h>
#include <tobas_msgs/Throttles.h>
#include <tobas_msgs/Battery.h>
#include <tobas_msgs/Wind.h>

#include "../include/tobas_gazebo_plugins/common.hpp"
#include "../include/tobas_gazebo_plugins/first_order_filter.hpp"

namespace gazebo
{
class GazeboRotorPlugin : public ModelPlugin
{
  // Constants
  static constexpr char kPluginName[] = "motor_model_plugin";
  static constexpr char kDebugTopicPrefix[] = "ground_truth/rotor_debug";
  static constexpr double kRotorSpeedCheckMargin = 10.;   // [rad/s]
  static constexpr double kTimeConstWarnThreshold = 0.1;  // [s]

  // Default parameters
  static constexpr double kDefaultMaxModelErrorRate = 0.;

  using self = GazeboRotorPlugin;
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
  int direction_;               // turning direction. 1(CCW) or -1(CW).
  SdfVector2 rot_speed_coefs_;  // [Vs/rad, (Vs/rad)^2]
  double motor_const_;
  double moment_const_;
  double rotor_drag_coef_;
  double max_model_error_rate_;
  double time_const_up_;
  double time_const_down_;
  double check_delay_threshold_;
  double auto_reset_time_thr_;

  double cmd_rot_speed_;  // [rad/s]
  tobas_msgs::BatteryConstPtr battery_;
  ignition::math::Vector3d wind_vel_W_ = zero3;  // [m/s]
  common::Time prev_sim_time_;
  common::Time last_cmd_time_;
  bool is_activated_ = false;
  bool is_initialized_ = false;
  bool battery_received_ = false;
  bool wind_received_ = false;
  AsymmetricFirstOrderFilter<double> rotor_speed_filter_;

  // Gazebo objects
  physics::ModelPtr model_;
  physics::JointPtr joint_;
  physics::LinkPtr link_;
  physics::LinkPtr parent_link_;
  event::ConnectionPtr update_connection_;

  // PubSub
  ros::Publisher debug_pub_;
  ros::Subscriber rotor_speeds_sub_;
  ros::Subscriber throttles_sub_;
  ros::Subscriber battery_sub_;
  ros::Subscriber wind_sub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate(const common::UpdateInfo& info);
  void registerPubSub();
  bool isReady();
  void applyForceAndTorque(const double& rot_speed, const common::Time cur_time);
  void updateRotationSpeed(const double& dt);
  double rotSpeedFromVoltage(const double& voltage);
  double maxRotSpeed();
  double minRotSpeed();
  void processCommandCommon(const size_t& data_size, const ros::Time& stamp);

  void rotorSpeedsCmdCb(const tobas_msgs::RotorSpeedsConstPtr& rotor_speeds);
  void throttlesCmdCb(const tobas_msgs::ThrottlesConstPtr& throttles);
  void batteryCb(const tobas_msgs::BatteryConstPtr& battery);
  void windSpeedCb(const tobas_msgs::WindConstPtr& wind);
};
}  // namespace gazebo

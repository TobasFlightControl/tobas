#pragma once

#include <map>
#include <functional>
#include <ros/ros.h>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>

#include <tobas_msgs/RotorState.h>

#include "../include/tobas_gazebo_plugins/common.hpp"

namespace gazebo
{
class GazeboBatteryPlugin : public ModelPlugin
{
  // Constants
  static constexpr char kPluginName[] = "battery_plugin";

  using self = GazeboBatteryPlugin;
  using super = ModelPlugin;

public:
  explicit GazeboBatteryPlugin();

protected:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  ros::NodeHandle nh_;

  // SDF parameters
  std::string ns_;
  double max_voltage_;   // [V] 満充電時の電圧
  double max_current_;   // [A] 最大電流
  size_t num_rotors_;

  std::vector<double> currents_;
  event::ConnectionPtr update_connection_;

  // Publishers
  ros::Publisher battery_pub_;

  // Subscribers
  std::vector<ros::Subscriber> rotor_state_subs_;
  std::vector<std::function<void(const tobas_msgs::RotorStateConstPtr&)>> rotor_state_cbs_;

  void getSdfParams(sdf::ElementPtr sdf);
  void registerPubSub();
  void onUpdate(const common::UpdateInfo& info);
};
}  // namespace gazebo

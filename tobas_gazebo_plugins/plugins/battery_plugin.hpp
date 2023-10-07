#pragma once

#include <ros/ros.h>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>

#include <tobas_msgs/Battery.h>

namespace gazebo
{
// Constants
static constexpr char kPluginName[] = "battery_plugin";

class GazeboBatteryPlugin : public ModelPlugin
{
  using super = ModelPlugin;

public:
  explicit GazeboBatteryPlugin();

protected:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  ros::NodeHandle nh_;

  // SDF parameters
  std::string ns_;
  double nominal_voltage_;  // 定格電圧

  tobas_msgs::Battery battery_msg_;
  event::ConnectionPtr update_connection_;

  // PubSub
  ros::Publisher battery_pub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void registerPubSub();
  void onUpdate(const common::UpdateInfo& info);
};
}  // namespace gazebo

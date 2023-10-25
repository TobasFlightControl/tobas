#pragma once

#include <ros/ros.h>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>

#include "../include/tobas_gazebo_plugins/common.hpp"

namespace gazebo
{
class GazeboBatteryPlugin : public ModelPlugin
{
  // Constants
  static constexpr char kPluginName[] = "battery_plugin";

  // Default parameters
  static constexpr double kDefaultVoltageNoiseStddev = 0.;

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
  double nominal_voltage_;  // 定格電圧
  double noise_stddev_;

  event::ConnectionPtr update_connection_;

  NormalDistribution noise_;
  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;

  // PubSub
  ros::Publisher battery_pub_;
  ros::Publisher battery_gt_pub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void registerPubSub();
  void onUpdate(const common::UpdateInfo& info);
};
}  // namespace gazebo

#pragma once

#include <random>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <sensor_msgs/FluidPressure.h>

#include "../include/tobas_gazebo_plugins/common.hpp"

namespace gazebo
{
// Constants
static constexpr char kPluginName[] = "barometer_plugin";

// Default values
static constexpr double kDefaultPressureVar = 1.;  // [Pa]

class GazeboBarometerPlugin : public SensorPlugin
{
  using super = SensorPlugin;

  using PressureMsg = sensor_msgs::FluidPressure;

public:
  explicit GazeboBarometerPlugin();

  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

private:
  ros::NodeHandle nh_;

  // SDF parameters
  std::string ns_;
  std::string link_name_;
  SdfVector3 offset_;  // B_Pos_BS
  double alt_0_;
  double pressure_var_;

  physics::WorldPtr world_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;
  sensor_msgs::FluidPressure pressure_msg_;

  NormalDistribution pressure_noise_;
  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;

  ros::Publisher pressure_pub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate();
};
}  // namespace gazebo

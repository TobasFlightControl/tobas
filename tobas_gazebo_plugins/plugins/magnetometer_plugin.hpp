#pragma once

#include <random>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <sensor_msgs/MagneticField.h>

#include "../include/tobas_gazebo_plugins/common.hpp"
#include "../include/tobas_gazebo_plugins/random.hpp"

namespace gazebo
{
// Constants
static const std::string kPluginName = "magnetometer_plugin";

// Default values
static const std::string kDefaultMagTopic = "magnetic_field";
static constexpr double kDefaultRefMagNorth = 3.0031e-05;
static constexpr double kDefaultRefMagEast = -4.116e-06;
static constexpr double kDefaultRefMagDown = 3.5615e-05;

class GazeboMagnetometerPlugin : public SensorPlugin
{
  using super = SensorPlugin;

  using MagMsg = sensor_msgs::MagneticField;

public:
  explicit GazeboMagnetometerPlugin();

  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

private:
  ros::NodeHandle nh_;

  // SDF parameters
  std::string ns_;
  std::string link_name_;
  std::string mag_topic_;
  double ref_mag_north_;
  double ref_mag_east_;
  double ref_mag_down_;
  SdfVector3 noise_normal_;
  SdfVector3 noise_uniform_initial_bias_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;
  ignition::math::Vector3d mag_NWU_;
  MagMsg mag_msg_;

  std::random_device rnd_dev_;
  NormalDistribution3dPtr noise_;

  ros::Publisher mag_pub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate();
};
}  // namespace gazebo

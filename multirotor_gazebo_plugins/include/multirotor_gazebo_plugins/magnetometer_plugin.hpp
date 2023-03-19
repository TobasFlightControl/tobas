#pragma once

#include <random>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <sensor_msgs/MagneticField.h>

namespace gazebo
{
// Constants
static constexpr char kPluginName[] = "magnetometer_plugin";

// Default values
static constexpr char kDefaultMagTopic[] = "magnetic_field";
static constexpr double kDefaultRefMagNorth = 3.0031e-05;
static constexpr double kDefaultRefMagEast = -4.116e-06;
static constexpr double kDefaultRefMagDown = 3.5615e-05;

class GazeboMagnetometerPlugin : public ModelPlugin
{
  using SdfVector3 = ignition::math::Vector3d;
  using NormalDistribution = std::normal_distribution<double>;
  using UniformDistribution = std::uniform_real_distribution<double>;
  using MagMsg = sensor_msgs::MagneticField;

public:
  GazeboMagnetometerPlugin();

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

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

  NormalDistribution noise_[3];
  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;

  ros::Publisher mag_pub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate(const common::UpdateInfo&);
};
}  // namespace gazebo

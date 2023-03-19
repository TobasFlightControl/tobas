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
// Default values
static constexpr char defaultMagTopic[] = "magnetic_field";
static constexpr double defaultRefMagNorth = 3.0031e-05;
static constexpr double defaultRefMagEast = -4.116e-06;
static constexpr double defaultRefMagDown = 3.5615e-05;

class GazeboMagnetometerPlugin : public ModelPlugin
{
  using SdfVector3 = ignition::math::Vector3d;
  using NormalDistribution = std::normal_distribution<double>;
  using UniformDistribution = std::uniform_real_distribution<double>;
  using MagMsg = sensor_msgs::MagneticField;

public:
  GazeboMagnetometerPlugin();

protected:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
  void getSdfParams(sdf::ElementPtr sdf);
  void OnUpdate(const common::UpdateInfo&);

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

  physics::WorldPtr world_;                 // Pointer to the world
  physics::ModelPtr model_;                 // Pointer to the model
  physics::LinkPtr link_;                   // Pointer to the link
  event::ConnectionPtr update_connection_;  // Pointer to the update event connection
  ignition::math::Vector3d mag_NWU_;        // Magnetic field in world NWU frame
  MagMsg mag_msg_;                          // Magnetic field message

  NormalDistribution noise_[3];
  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;

  ros::Publisher mag_pub_;
};
}  // namespace gazebo

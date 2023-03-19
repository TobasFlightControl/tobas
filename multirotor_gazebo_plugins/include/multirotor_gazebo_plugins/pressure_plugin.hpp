#pragma once

#include <random>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <sensor_msgs/FluidPressure.h>

namespace gazebo
{
// Constants
static constexpr double kGasConstantNmPerKmolKelvin = 8314.32;
static constexpr double kMeanMolecularAirWeightKgPerKmol = 28.9644;
static constexpr double kGravityMagnitude = 9.80665;
static constexpr double kEarthRadiusMeters = 6356766.0;
static constexpr double kPressureOneAtmospherePascals = 101325.0;
static constexpr double kSeaLevelTempKelvin = 288.15;
static constexpr double kTempLapseKelvinPerMeter = 0.0065;
static constexpr double kAirConstantDimensionless =
  kGravityMagnitude * kMeanMolecularAirWeightKgPerKmol
  / (kGasConstantNmPerKmolKelvin * -kTempLapseKelvinPerMeter);
static constexpr char kPluginName[] = "pressure_plugin";

// Default values
static const std::string kDefaultPressurePubTopic = "air_pressure";
static constexpr double kDefaultRefAlt = 500.;
static constexpr double kDefaultPressureVar = 1.;

class GazeboPressurePlugin : public ModelPlugin
{
  using NormalDistribution = std::normal_distribution<double>;
  using PressureMsg = sensor_msgs::FluidPressure;

public:
  GazeboPressurePlugin();

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

private:
  ros::NodeHandle nh_;

  // SDF parameters
  std::string ns_;
  std::string link_name_;
  std::string pressure_topic_;
  double ref_alt_;
  double pressure_var_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;
  sensor_msgs::FluidPressure pressure_msg_;

  NormalDistribution pressure_noise_;
  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;

  ros::Publisher pressure_pub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate(const common::UpdateInfo&);
};
}  // namespace gazebo

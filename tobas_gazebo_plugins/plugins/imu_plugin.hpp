#pragma once

#include <random>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <sensor_msgs/Imu.h>

#include <tobas_tools/constants.hpp>

#include "../include/tobas_gazebo_plugins/common.hpp"

namespace gazebo
{
// Constants
static const std::string kPluginName = "imu_plugin";

// Default values
static const std::string kDefaultImuTopic = "imu";
static const std::string kDefaultDebugTopic = "ground_truth/imu_debug";
static constexpr double kDefaultGyroNoiseDensity = 2. * 35. / 3600. * kDegreeToRadian;
static constexpr double kDefaultGyroRandomWalk = 2. * 4. / 3600. * kDegreeToRadian;
static constexpr double kDefaultGyroBiasCorrTime = 1000.;
static constexpr double kDefaultGyroTurnOnBiasSigma = 0.5 * kDegreeToRadian;
static constexpr double kDefaultAccNoiseDensity = 2. * 2e-3;
static constexpr double kDefaultAccRandomWalk = 2. * 3e-3;
static constexpr double kDefaultAccBiasCorrTime = 300.;
static constexpr double kDefaultAccTurnOnBiasSigma = 2e-2 * tobas::kGravity;

class GazeboImuPlugin : public SensorPlugin
{
  using super = SensorPlugin;

public:
  explicit GazeboImuPlugin();

  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

private:
  ros::NodeHandle nh_;

  // SDF parameters
  std::string ns_;
  std::string link_name_;
  std::string imu_topic_;
  std::string debug_topic_;
  SdfVector3 offset_;               // B_Pos_BS
  double gyro_noise_density_sig_;   // Gyro noise density actually added to signal [rad/s/sqrt(Hz)]
  double gyro_noise_density_obs_;   // Gyro noise density that is observed [rad/s/sqrt(Hz)]
  double gyro_random_walk_;         // Gyro bias random walk [rad/s/s/sqrt(Hz)]
  double gyro_bias_corr_time_;      // Gyro bias correlation time constant [s]
  double gyro_turn_on_bias_sigma_;  // Gyro turn on bias standard deviation [rad/s]
  double acc_noise_density_sig_;    // Accel noise density actually added to signal [m/s^2/sqrt(Hz)]
  double acc_noise_density_obs_;    // Accel noise density that is observerd [m/s^2/sqrt(Hz)]
  double acc_random_walk_;          // Accel bias random walk [m/s^2/s/sqrt(Hz)]
  double acc_bias_corr_time_;       // Accel bias correlation time constant [s]
  double acc_turn_on_bias_sigma_;   // Accel turn on bias standard deviation [m/s^2]

  physics::WorldPtr world_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;
  common::Time last_time_;
  sensor_msgs::Imu imu_msg_;
  tobas_gazebo_plugins::ImuDebug debug_msg_;
  ignition::math::Vector3d gyro_bias_;
  ignition::math::Vector3d acc_bias_;
  ignition::math::Vector3d gyro_turn_on_bias_;
  ignition::math::Vector3d acc_turn_on_bias_;

  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;
  NormalDistribution noise_;

  ros::Publisher imu_pub_;
  ros::Publisher debug_pub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate();
  void addNoise(
    ignition::math::Vector3d& acc_meas,
    ignition::math::Vector3d& gyro_meas,
    const double& dt);
};
}  // namespace gazebo

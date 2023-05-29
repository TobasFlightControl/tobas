#pragma once

#include <random>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <sensor_msgs/Imu.h>

#include "../tobas_gazebo_plugins/common.hpp"

namespace gazebo
{
// Constants
static const std::string kPluginName = "imu_plugin";

// Default values
static const std::string kDefaultImuTopic = "imu";
static constexpr double kDefaultGyroNoiseDensity = 2. * 35. / 3600. / 180. * M_PI;
static constexpr double kDefaultGyroRandomWalk = 2. * 4. / 3600. / 180. * M_PI;
static constexpr double kDefaultGyroBiasCorrTime = 1e+3;
static constexpr double kDefaultGyroTurnOnBiasSigma = 0.5 / 180.0 * M_PI;
static constexpr double kDefaultAccNoiseDensity = 2. * 2e-3;
static constexpr double kDefaultAccRandomWalk = 2. * 3e-3;
static constexpr double kDefaultAccBiasCorrTime = 300.;
static constexpr double kDefaultAccTurnOnBiasSigma = 2e-2 * 9.80665;

class GazeboImuPlugin : public SensorPlugin
{
  using super = SensorPlugin;

  using ImuMsg = sensor_msgs::Imu;

public:
  explicit GazeboImuPlugin();

  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

private:
  ros::NodeHandle nh_;

  // SDF parameters
  std::string ns_;
  std::string link_name_;
  std::string imu_topic_;
  double gyro_noise_density_;       // Gyroscope noise density spectrum [rad/s/sqrt(Hz)]
  double gyro_random_walk_;         // Gyroscope bias random walk [rad/s/s/sqrt(Hz)]
  double gyro_bias_corr_time_;      // Gyroscope bias correlation time constant [s]
  double gyro_turn_on_bias_sigma_;  // Gyroscope turn on bias standard deviation [rad/s]
  double acc_noise_density_;        // Accelerometer noise density spectrum [m/s^2/sqrt(Hz)]
  double acc_random_walk_;          // Accelerometer bias random walk [m/s^2/s/sqrt(Hz)]
  double acc_bias_corr_time_;       // Accelerometer bias correlation time constant [s]
  double acc_turn_on_bias_sigma_;   // Accelerometer turn on bias standard deviation [m/s^2]

  physics::WorldPtr world_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;
  common::Time last_time_;
  ImuMsg imu_msg_;
  ignition::math::Vector3d gravity_W_;
  ignition::math::Vector3d velocity_prev_W_;
  ignition::math::Vector3d gyro_bias_;
  ignition::math::Vector3d acc_bias_;
  ignition::math::Vector3d gyro_turn_on_bias_;
  ignition::math::Vector3d acc_turn_on_bias_;

  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;
  NormalDistribution noise_;

  ros::Publisher imu_pub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate();
  void addNoise(ignition::math::Vector3d& lin_acc, ignition::math::Vector3d& ang_vel, double dt);
};
}  // namespace gazebo

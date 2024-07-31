#pragma once

#include <random>
#include <rclcpp/rclcpp.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

#include <tobas_dsp/low_pass_filter.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_gazebo_plugins/common.hpp"

namespace gazebo
{
class GazeboImuPlugin : public SensorPlugin
{
  // Constants
  static constexpr char kPluginName[] = "imu_plugin";
  static constexpr char kDebugPubTopic[] = "ground_truth/imu_debug";

  // Default values
  static constexpr double kDefaultAccNoiseDensity = 2. * 2e-3;
  static constexpr double kDefaultAccRandomWalk = 2. * 3e-3;
  static constexpr double kDefaultAccBiasCorrTime = 300.;
  static constexpr double kDefaultAccTurnOnBiasSigma = 2e-2 * tobas::kGravity;
  static constexpr double kDefaultAccLpfCutoffFreq = 20.;
  static constexpr double kDefaultGyroNoiseDensity = 2. * 35. / 3600. * tobas::kDeg2Rad;
  static constexpr double kDefaultGyroRandomWalk = 2. * 4. / 3600. * tobas::kDeg2Rad;
  static constexpr double kDefaultGyroBiasCorrTime = 1000.;
  static constexpr double kDefaultGyroTurnOnBiasSigma = 0.5 * tobas::kDeg2Rad;
  static constexpr double kDefaultGyroLpfCutoffFreq = 20.;

  using self = GazeboImuPlugin;
  using super = SensorPlugin;

public:
  explicit GazeboImuPlugin();

  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

private:
  rclcpp::NodeHandle nh_;

  // SDF parameters
  std::string ns_;
  std::string link_name_;
  SdfVector3 offset_;               // B_Pos_BS
  double acc_noise_density_sig_;    // Accel noise density actually added to signal [m/s^2/sqrt(Hz)]
  double acc_noise_density_obs_;    // Accel noise density that is observerd [m/s^2/sqrt(Hz)]
  double acc_random_walk_;          // Accel bias random walk [m/s^2/s/sqrt(Hz)]
  double acc_bias_corr_time_;       // Accel bias correlation time constant [s]
  double acc_turn_on_bias_sigma_;   // Accel turn on bias standard deviation [m/s^2]
  double acc_lpf_cutoff_freq_;      // LPF cutoff frequency for accelerometer [Hz]
  double gyro_noise_density_sig_;   // Gyro noise density actually added to signal [rad/s/sqrt(Hz)]
  double gyro_noise_density_obs_;   // Gyro noise density that is observed [rad/s/sqrt(Hz)]
  double gyro_random_walk_;         // Gyro bias random walk [rad/s/s/sqrt(Hz)]
  double gyro_bias_corr_time_;      // Gyro bias correlation time constant [s]
  double gyro_turn_on_bias_sigma_;  // Gyro turn on bias standard deviation [rad/s]
  double gyro_lpf_cutoff_freq_;     // LPF cutoff frequency for gyroscope [Hz]

  physics::WorldPtr world_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;
  common::Time last_time_ = common::Time(0);
  ignition::math::Vector3d acc_bias_ = zero3, gyro_bias_ = zero3;
  ignition::math::Vector3d acc_turn_on_bias_, gyro_turn_on_bias_;
  dsp::LowPassFilter<ignition::math::Vector3d> acc_lpf_, gyro_lpf_;  // Internal LPF

  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;
  NormalDistribution noise_;

  rclcpp::Publisher imu_pub_;
  rclcpp::Publisher debug_pub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate();
  void addNoise(ignition::math::Vector3d& acc, ignition::math::Vector3d& gyro, const double& dt);
  void publishImuMsg(const double& dt) const;
  void publishDebugMsg() const;
};
}  // namespace gazebo

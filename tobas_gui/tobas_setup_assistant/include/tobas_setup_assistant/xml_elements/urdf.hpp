#pragma once

#include <eigen3/Eigen/Core>
#include <tinyxml2.h>

#include <tobas_drone_core/drone.hpp>

namespace gui
{
namespace setup_assistant
{
void addBatteryPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  double update_rate,
  double max_voltage,
  double sag_voltage,
  double max_current,
  double current_capacity,
  double internal_registance,
  int num_rotors);

void addIMUPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& link_name,
  double update_rate,
  const Eigen::Vector3d& offset,
  double gyro_noise_density,
  double gyro_random_walk,
  double gyro_bias_corr_time,
  double gyro_turn_on_bias_sigma,
  double gyro_lpf_cutoff_freq,
  double acc_noise_density,
  double acc_random_walk,
  double acc_bias_corr_time,
  double acc_turn_on_bias_sigma,
  double acc_lpf_cutoff_freq);

void addMagnetometerPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& link_name,
  double update_rate,
  const Eigen::Vector3d& offset,
  double latitude_zero,
  double longitude_zero,
  double altitude_zero,
  double gauss_noise,
  double uniform_noise);

void addBarometerPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& link_name,
  double update_rate,
  const Eigen::Vector3d& offset,
  double altitude_zero,
  double pressure_variance);

void addGPSPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& link_name,
  double update_rate,
  const Eigen::Vector3d& offset,
  double delay,
  double position_corr_time,
  double hor_pos_accuracy,
  double ver_pos_accuracy,
  double hor_vel_stddev,
  double ver_vel_stddev,
  double latitude_zero,
  double longitude_zero,
  double altitude_zero);

void addRotorPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& joint_name,
  const tobas::RotorConfig& rotor,
  double time_const_up,
  double time_const_down,
  double max_current,
  double max_model_error_rate);

void addFixedWingPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& link_name,
  double altitude_zero,
  const tobas::FixedWingConfig& fixed_wing);

void addGazeboWindPlugin(tinyxml2::XMLElement* robot, const std::string& ns, const std::string& link_name);

void addGazeboGroundTruthStatePlugin(tinyxml2::XMLElement* robot, const std::string& ns, const std::string& link_name);

void addRotorSpeedsPublisherPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::vector<std::string>& rotor_joint_names);

/* https://github.com/ros-controls/gz_ros2_control/tree/jazzy */
void addGazeboSimROS2ControlPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& pkg_name,
  const std::string& params_rel_path);

void addGazeboROS2SimSystem(tinyxml2::XMLElement* robot, const tobas::JointConfigMap& joints);

void addBaseStaticJoint(tinyxml2::XMLElement* robot, const std::string& root_link_name);
}  // namespace setup_assistant
}  // namespace gui

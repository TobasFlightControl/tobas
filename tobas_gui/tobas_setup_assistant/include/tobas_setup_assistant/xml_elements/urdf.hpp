#pragma once

#include <eigen3/Eigen/Core>
#include <tinyxml2.h>

#include <tobas_drone_core/drone.hpp>

namespace gui
{
namespace sa
{
namespace xml
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
  const std::vector<std::string>& rotor_link_names);

void addIMUPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& link_name,
  double update_rate,
  const Eigen::Vector3d& offset,
  double gyro_noise_density,
  double gyro_offset_norm,
  double gyro_random_walk,
  double gyro_bias_corr_time,
  double acc_noise_density,
  double acc_offset_norm,
  double acc_random_walk,
  double acc_bias_corr_time,
  const std::vector<std::string>& rotor_link_names);

void addMagnetometerPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& link_name,
  double update_rate,
  const Eigen::Vector3d& offset,
  double latitude_zero,
  double longitude_zero,
  double altitude_zero,
  double noise_stddev,
  double hard_bias_norm);

void addBarometerPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& link_name,
  double update_rate,
  const Eigen::Vector3d& offset,
  double altitude_zero,
  double pressure_variance);

void addGNSSPlugin(
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

void addElectricPropulsionSystemPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& link_name,
  double kv,
  double internal_resistance,
  size_t num_blades,
  double motor_const,
  double moment_const,
  double drag_const,
  tobas::turning_direction_t direction,
  double max_current,
  double max_model_error_rate);

struct EngineParam
{
  double torque_const;
  double friction_torque;
  double time_const_up;
  double time_const_down;
};

struct ICERotorParam
{
  std::string link_name;
  tobas::turning_direction_t direction;
  double gear_ratio;
  size_t num_blades;
  tobas_std::Range<double> pitch_angle_limit;
  double max_pitch_angle_rate;
  std::pair<double, double> motor_const;
  double moment_const;
  std::pair<double, double> drag_const;
};

void addICEPropulsionSystemPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const EngineParam& engine_param,
  const std::vector<ICERotorParam>& rotor_params);

void addFixedWingPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& link_name,
  double altitude_zero,
  const tobas::FixedWingConfig& fixed_wing);

void addGazeboWindPlugin(tinyxml2::XMLElement* robot, const std::string& ns, const std::string& link_name);

void addGazeboGroundTruthStatePlugin(tinyxml2::XMLElement* robot, const std::string& ns, const std::string& link_name);

void addGazeboLookAtPositionPlugin(tinyxml2::XMLElement* robot, const std::string& ns, const std::string& link_name);

/* https://github.com/ros-controls/gz_ros2_control/tree/jazzy */
void addGazeboSimROS2ControlPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& pkg_name,
  const std::string& params_rel_path);

void addGazeboROS2SimSystem(tinyxml2::XMLElement* robot, const tobas::JointConfigMap& joints);

void addBaseStaticJoint(tinyxml2::XMLElement* robot, const std::string& root_link_name);
}  // namespace xml
}  // namespace sa
}  // namespace gui

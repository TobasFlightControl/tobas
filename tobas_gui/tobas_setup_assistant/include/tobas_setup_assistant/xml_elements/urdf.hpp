#pragma once

#include <tinyxml2.h>
#include <eigen3/Eigen/Core>

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
  int update_rate,
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
  int update_rate,
  const Eigen::Vector3d& offset,
  double gyro_noise_density,
  double gyro_random_walk,
  double gyro_bias_corr_time,
  double acc_noise_density,
  double acc_random_walk,
  double acc_bias_corr_time,
  const std::vector<std::string>& rotor_link_names);

void addMagnetometerPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& link_name,
  int update_rate,
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
  int update_rate,
  const Eigen::Vector3d& offset,
  double altitude_zero,
  double noise_stddev);

void addGNSSPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& link_name,
  int update_rate,
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
  tobas::TurningDirection direction,
  double max_current,
  double max_model_error_rate);

struct EngineParam
{
  std::pair<double, double> engine_const;
  double time_const_up;
  double time_const_down;
};

struct IceRotorParam
{
  std::string link_name;
  tobas::TurningDirection direction;
  double gear_ratio;
  size_t num_blades;
  tobas_std::Range<double> pitch_angle_limit;  // [rad]
  double max_pitch_angle_rate;                 // [rad/s]
  std::pair<double, double> motor_const;
  double moment_const;
  std::pair<double, double> drag_const;
};

void addIcePropulsionSystemPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const EngineParam& engine_param,
  const std::vector<IceRotorParam>& rotor_params);

void addFixedWingPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& base_link_name,
  double altitude_zero,
  const tobas::FixedWingConfig& fixed_wing);

void addJointStateBroadcasterPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::vector<std::string>& joint_names,
  int update_rate);

void addJointPositionControllerPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& joint_name,
  double home_pos,
  double time_const);

void addJointVelocityControllerPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& joint_name,
  double home_pos);

void addJointEffortControllerPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& joint_name,
  double home_pos);

void addGazeboWindPlugin(tinyxml2::XMLElement* robot, const std::string& ns, const std::string& link_name);

void addGazeboGroundTruthStatePlugin(tinyxml2::XMLElement* robot, const std::string& ns, const std::string& link_name);

void addGazeboLookAtPositionPlugin(tinyxml2::XMLElement* robot, const std::string& ns, const std::string& link_name);

void addBaseStaticJoint(tinyxml2::XMLElement* robot, const std::string& root_link_name);
}  // namespace xml
}  // namespace sa
}  // namespace gui

#pragma once

#include <eigen3/Eigen/Core>
#include <tinyxml2.h>
#include <QStringList>

#include <tobas_drone_core/control_surface.hpp>

namespace gui
{
namespace setup_assistant
{
void addBatteryPlugin(
  tinyxml2::XMLElement* robot,
  const char* ns,
  double update_rate,
  double max_voltage,
  double sag_voltage,
  double max_current,
  double current_capacity,
  double internal_registance,
  int num_rotors);

void addIMUPlugin(
  tinyxml2::XMLElement* robot,
  const char* ns,
  const char* link_name,
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
  const char* ns,
  const char* link_name,
  double update_rate,
  const Eigen::Vector3d& offset,
  double latitude_zero,
  double longitude_zero,
  double altitude_zero,
  double gauss_noise,
  double uniform_noise);

void addBarometerPlugin(
  tinyxml2::XMLElement* robot,
  const char* ns,
  const char* link_name,
  double update_rate,
  const Eigen::Vector3d& offset,
  double altitude_zero,
  double pressure_variance);

void addGPSPlugin(
  tinyxml2::XMLElement* robot,
  const char* ns,
  const char* link_name,
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
  const char* ns,
  int channel,
  const char* joint_name,
  const std::pair<double, double>& rot_speed_coefs,
  double motor_const,
  double moment_const,
  double rotor_drag_coef,
  const char* turning_direction,
  double time_const_up,
  double time_const_down,
  double max_rot_speed,
  int num_poles,
  double max_current,
  const char* esc_mode,
  double max_model_error_rate);

void addFixedWingPlugin(
  tinyxml2::XMLElement* robot,
  const char* ns,
  const char* link_name,
  double altitude_zero,
  double wing_surface,
  double wing_span,
  double mean_aerodynamic_chord,
  const Eigen::Vector3d& aerodynamic_center,
  std::pair<double, double> alpha_limit,
  double c_lift_0,
  double c_lift_alpha,
  double c_drag_0,
  double c_drag_alpha,
  double c_side_beta,
  double c_roll_beta,
  double c_roll_p,
  double c_roll_r,
  double c_pitch_0,
  double c_pitch_alpha,
  double c_pitch_abs_beta,
  double c_pitch_alpha_rate,
  double c_pitch_q,
  double c_yaw_beta,
  double c_yaw_p,
  double c_yaw_r,
  const tobas::ControlSurfaces& control_surfaces);

void addGazeboWindPlugin(tinyxml2::XMLElement* robot, const char* ns, const char* link_name);

void addGazeboGroundTruthStatePlugin(tinyxml2::XMLElement* robot, const char* ns, const char* link_name);

void addRotorSpeedsPublisherPlugin(tinyxml2::XMLElement* robot, const char* ns, const QStringList& rotor_joint_names);
}  // namespace setup_assistant
}  // namespace gui

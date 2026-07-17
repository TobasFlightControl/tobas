// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tinyxml2.h>
#include <eigen3/Eigen/Core>

#include <tobas_drone_core/fixed_wing/fixed_wing.hpp>
#include <tobas_drone_core/propulsion_system/ice_propulsion_system/aerodynamics.hpp>
#include <tobas_drone_core/propulsion_system/turning_direction.hpp>

namespace tobas
{
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

void addImuPlugin(
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
  double noise_stddev,
  double hard_bias_norm);

void addBarometerPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& link_name,
  int update_rate,
  const Eigen::Vector3d& offset,
  double noise_stddev);

void addGnssPlugin(
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
  double geoid_undulation);

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
  TurningDirection direction,
  double max_current);

struct EngineParam
{
  std::pair<double, double> engine_const;
  double time_const_up;
  double time_const_down;
};

struct IceRotorParam
{
  std::string link_name;
  TurningDirection direction;
  double gear_ratio;
  size_t num_blades;
  st::Range<double> pitch_angle_limit;  // [rad]
  double max_pitch_angle_rate;          // [rad/s]
  VppMotorConstant motor_const;
  VppMomentConstant moment_const;
  VppDragConstant drag_const;
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
  const FixedWingConfig& fixed_wing);

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

void addGazeboSuspendedLoadPlugin(tinyxml2::XMLElement* robot, const std::string& ns, const std::string& link_name);

void addBaseStaticJoint(tinyxml2::XMLElement* robot, const std::string& root_link_name);
}  // namespace xml
}  // namespace sa
}  // namespace gui
}  // namespace tobas

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tinyxml2.h>
#include <eigen3/Eigen/Core>

#include <tobas_drone_core/fixed_wing/fixed_wing.hpp>
#include <tobas_drone_core/propulsion_system/ice_propulsion_system/aerodynamics.hpp>
#include <tobas_drone_core/propulsion_system/turning_direction.hpp>

namespace tobas::gui::sa::xml
{
void addBatteryPlugin(
  tinyxml2::XMLElement* _robot,
  const std::string& _ns,
  int _update_rate,
  double _max_voltage,
  double _sag_voltage,
  double _max_current,
  double _current_capacity,
  double _internal_registance,
  const std::vector<std::string>& _rotor_link_names);

void addImuPlugin(
  tinyxml2::XMLElement* _robot,
  const std::string& _ns,
  const std::string& _link_name,
  const Eigen::Vector3d& _offset,
  int _update_rate,
  double _gyro_noise_density,
  double _gyro_random_walk,
  double _gyro_bias_corr_time,
  double _acc_noise_density,
  double _acc_random_walk,
  double _acc_bias_corr_time,
  const std::vector<std::string>& _rotor_link_names);

void addMagnetometerPlugin(
  tinyxml2::XMLElement* _robot,
  const std::string& _ns,
  const std::string& _link_name,
  const Eigen::Vector3d& _offset,
  int _update_rate,
  double _noise_stddev,
  double _hard_bias_norm);

void addBarometerPlugin(
  tinyxml2::XMLElement* _robot,
  const std::string& _ns,
  const std::string& _link_name,
  const Eigen::Vector3d& _offset,
  int _update_rate,
  double _noise_stddev);

void addGnssPlugin(
  tinyxml2::XMLElement* _robot,
  const std::string& _ns,
  const std::string& _link_name,
  const Eigen::Vector3d& _offset,
  int _update_rate,
  double _delay,
  double _position_corr_time,
  double _hor_pos_accuracy,
  double _ver_pos_accuracy,
  double _hor_vel_stddev,
  double _ver_vel_stddev,
  double _geoid_undulation);

void addElectricPropulsionSystemPlugin(
  tinyxml2::XMLElement* _robot,
  const std::string& _ns,
  const std::string& _link_name,
  double _kv,
  double _internal_resistance,
  size_t _num_blades,
  double _motor_const,
  double _moment_const,
  double _drag_const,
  TurningDirection _direction,
  double _max_cont_current);

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
  st::Range<double> pitch_angle_limit;  ///< [rad]
  double max_pitch_angle_rate;          ///< [rad/s]
  VppMotorConstant motor_const;
  VppMomentConstant moment_const;
  VppDragConstant drag_const;
};

void addIcePropulsionSystemPlugin(
  tinyxml2::XMLElement* _robot,
  const std::string& _ns,
  const EngineParam& _engine_param,
  const std::vector<IceRotorParam>& _rotor_params);

void addFixedWingPlugin(
  tinyxml2::XMLElement* _robot,
  const std::string& _ns,
  const std::string& _base_link_name,
  const FixedWingConfig& _fixed_wing);

void addJointStateBroadcasterPlugin(
  tinyxml2::XMLElement* _robot,
  const std::string& _ns,
  const std::vector<std::string>& _joint_names,
  int _update_rate);

void addJointPositionControllerPlugin(
  tinyxml2::XMLElement* _robot,
  const std::string& _ns,
  const std::string& _joint_name,
  double _home_pos,
  double _time_const);

void addJointVelocityControllerPlugin(
  tinyxml2::XMLElement* _robot,
  const std::string& _ns,
  const std::string& _joint_name,
  double _home_pos);

void addJointEffortControllerPlugin(
  tinyxml2::XMLElement* _robot,
  const std::string& _ns,
  const std::string& _joint_name,
  double _home_pos);

void addGazeboWindPlugin(tinyxml2::XMLElement* _robot, const std::string& _ns, const std::string& _link_name);

void addGazeboGroundTruthStatePlugin(tinyxml2::XMLElement* _robot, const std::string& _ns, const std::string& _link_name);

void addGazeboLookAtPositionPlugin(tinyxml2::XMLElement* _robot, const std::string& _ns, const std::string& _link_name);

void addGazeboFixedLoadPlugin(tinyxml2::XMLElement* _robot, const std::string& _ns, const std::string& _link_name);

void addGazeboSuspendedLoadPlugin(tinyxml2::XMLElement* _robot, const std::string& _ns, const std::string& _link_name);

void addBaseStaticJoint(tinyxml2::XMLElement* _robot, const std::string& _root_link_name);
}  // namespace tobas::gui::sa::xml

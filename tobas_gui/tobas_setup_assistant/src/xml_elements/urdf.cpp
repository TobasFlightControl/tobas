// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/xml_elements/urdf.hpp"

#include <format>
#include <ranges>
#include <type_traits>

#include <tobas_constants/frame.hpp>

namespace tobas::gui::sa::xml
{
namespace
{
template <typename T>
  requires std::is_arithmetic_v<T>
std::string toString(T _data)
{
  return std::to_string(_data);
}

template <typename T>
  requires(!std::is_arithmetic_v<T>)
std::string toString(const T& _data)
{
  return std::to_string(_data);
}

template <>
std::string toString<std::string>(const std::string& _data)
{
  return _data;
}

template <>
std::string toString<double>(double _data)
{
  return std::format("{:.{}g}", _data, 9);
}

template <>
std::string toString<std::vector<double>>(const std::vector<double>& _data)
{
  std::string res;
  for (const auto& [i, x] : std::views::enumerate(_data)) {
    if (i != 0) {
      res += ' ';
    }
    res += toString(x);
  }
  return res;
}

template <>
std::string toString<std::pair<double, double>>(const std::pair<double, double>& _data)
{
  // `format("{} {}", first, second)` may cause mojibake, so convert each character to a string.
  return toString(_data.first) + ' ' + toString(_data.second);
}

template <>
std::string toString<Eigen::Vector3d>(const Eigen::Vector3d& _data)
{
  return toString(_data.x()) + ' ' + toString(_data.y()) + ' ' + toString(_data.z());
}

template <typename T>
void addList(tinyxml2::XMLElement* _parent, const std::string& _list_name, const std::vector<T>& _items)
{
  const auto elem = _parent->InsertNewChildElement(_list_name.c_str());
  for (const auto& item : _items) {
    elem->InsertNewChildElement("item")->SetText(toString(item).c_str());
  }
}

tinyxml2::XMLElement*
addGazeboPlugin(tinyxml2::XMLElement* _robot, const std::string& _filename, const std::string& _name)
{
  // robot/gazebo
  const auto gazebo = _robot->InsertNewChildElement("gazebo");

  // robot/gazebo/plugin
  const auto plugin = gazebo->InsertNewChildElement("plugin");
  plugin->SetAttribute("filename", _filename.c_str());
  plugin->SetAttribute("name", _name.c_str());

  return plugin;
}
}  // namespace

void addBatteryPlugin(
  tinyxml2::XMLElement* _robot,
  const std::string& _ns,
  int _update_rate,
  double _max_voltage,
  double _sag_voltage,
  double _max_current,
  double _current_capacity,
  double _internal_registance,
  const std::vector<std::string>& _rotor_link_names)
{
  const auto plugin = addGazeboPlugin(_robot, "tobas_gazebo_battery_plugin", "tobas::gazebo::GazeboBatteryPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(_ns.c_str());
  plugin->InsertNewChildElement("updateRate")->SetText(toString(_update_rate).c_str());
  plugin->InsertNewChildElement("maxVoltage")->SetText(toString(_max_voltage).c_str());
  plugin->InsertNewChildElement("sagVoltage")->SetText(toString(_sag_voltage).c_str());
  plugin->InsertNewChildElement("maxCurrent")->SetText(toString(_max_current).c_str());
  plugin->InsertNewChildElement("currentCapacity")->SetText(toString(_current_capacity).c_str());
  plugin->InsertNewChildElement("internalRegistance")->SetText(toString(_internal_registance).c_str());
  addList(plugin, "rotorLinkNames", _rotor_link_names);
}

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
  const std::vector<std::string>& _rotor_link_names)
{
  const auto plugin = addGazeboPlugin(_robot, "tobas_gazebo_imu_plugin", "tobas::gazebo::GazeboImuPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(_ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(_link_name.c_str());
  plugin->InsertNewChildElement("offset")->SetText(toString(_offset).c_str());
  plugin->InsertNewChildElement("updateRate")->SetText(toString(_update_rate).c_str());
  plugin->InsertNewChildElement("gyroNoiseDensity")->SetText(toString(_gyro_noise_density).c_str());
  plugin->InsertNewChildElement("gyroRandomWalk")->SetText(toString(_gyro_random_walk).c_str());
  plugin->InsertNewChildElement("gyroBiasCorrelationTime")->SetText(toString(_gyro_bias_corr_time).c_str());
  plugin->InsertNewChildElement("accelNoiseDensity")->SetText(toString(_acc_noise_density).c_str());
  plugin->InsertNewChildElement("accelRandomWalk")->SetText(toString(_acc_random_walk).c_str());
  plugin->InsertNewChildElement("accelBiasCorrelationTime")->SetText(toString(_acc_bias_corr_time).c_str());
  addList(plugin, "rotorLinkNames", _rotor_link_names);
}

void addMagnetometerPlugin(
  tinyxml2::XMLElement* _robot,
  const std::string& _ns,
  const std::string& _link_name,
  const Eigen::Vector3d& _offset,
  int _update_rate,
  double _noise_stddev,
  double _hard_bias_norm)
{
  const auto plugin =
    addGazeboPlugin(_robot, "tobas_gazebo_magnetometer_plugin", "tobas::gazebo::GazeboMagnetometerPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(_ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(_link_name.c_str());
  plugin->InsertNewChildElement("offset")->SetText(toString(_offset).c_str());
  plugin->InsertNewChildElement("updateRate")->SetText(toString(_update_rate).c_str());
  plugin->InsertNewChildElement("noiseStddev")->SetText(toString(_noise_stddev).c_str());
  plugin->InsertNewChildElement("hardBiasNorm")->SetText(toString(_hard_bias_norm).c_str());
}

void addBarometerPlugin(
  tinyxml2::XMLElement* _robot,
  const std::string& _ns,
  const std::string& _link_name,
  const Eigen::Vector3d& _offset,
  int _update_rate,
  double _noise_stddev)
{
  const auto plugin = addGazeboPlugin(_robot, "tobas_gazebo_barometer_plugin", "tobas::gazebo::GazeboBarometerPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(_ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(_link_name.c_str());
  plugin->InsertNewChildElement("offset")->SetText(toString(_offset).c_str());
  plugin->InsertNewChildElement("updateRate")->SetText(toString(_update_rate).c_str());
  plugin->InsertNewChildElement("noiseStddev")->SetText(toString(_noise_stddev).c_str());
}

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
  double _ver_vel_stddev,
  double _hor_vel_stddev,
  double _geoid_undulation)
{
  const auto plugin = addGazeboPlugin(_robot, "tobas_gazebo_gnss_plugin", "tobas::gazebo::GazeboGnssPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(_ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(_link_name.c_str());
  plugin->InsertNewChildElement("offset")->SetText(toString(_offset).c_str());
  plugin->InsertNewChildElement("updateRate")->SetText(toString(_update_rate).c_str());
  plugin->InsertNewChildElement("delay")->SetText(toString(_delay).c_str());
  plugin->InsertNewChildElement("positionCorrTime")->SetText(toString(_position_corr_time).c_str());
  plugin->InsertNewChildElement("horPosAccuracy")->SetText(toString(_hor_pos_accuracy).c_str());
  plugin->InsertNewChildElement("verPosAccuracy")->SetText(toString(_ver_pos_accuracy).c_str());
  plugin->InsertNewChildElement("horVelStdDev")->SetText(toString(_hor_vel_stddev).c_str());
  plugin->InsertNewChildElement("verVelStdDev")->SetText(toString(_ver_vel_stddev).c_str());
  plugin->InsertNewChildElement("geoidUndulation")->SetText(toString(_geoid_undulation).c_str());
}

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
  double _max_cont_current)
{
  const auto plugin = addGazeboPlugin(
    _robot, "tobas_gazebo_electric_propulsion_system_plugin", "tobas::gazebo::GazeboElectricPropulsionSystemPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(_ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(_link_name.c_str());
  plugin->InsertNewChildElement("kv")->SetText(toString(_kv).c_str());
  plugin->InsertNewChildElement("internalResistance")->SetText(_internal_resistance);
  plugin->InsertNewChildElement("numberOfBlades")->SetText(toString(_num_blades).c_str());
  plugin->InsertNewChildElement("motorConstant")->SetText(toString(_motor_const).c_str());
  plugin->InsertNewChildElement("momentConstant")->SetText(toString(_moment_const).c_str());
  plugin->InsertNewChildElement("dragConstant")->SetText(toString(_drag_const).c_str());
  plugin->InsertNewChildElement("turningDirection")->SetText(textFromEnum(_direction).c_str());
  plugin->InsertNewChildElement("maxContinuousCurrent")->SetText(toString(_max_cont_current).c_str());
}

void addIcePropulsionSystemPlugin(
  tinyxml2::XMLElement* _robot,
  const std::string& _ns,
  const EngineParam& _engine_param,
  const std::vector<IceRotorParam>& _rotor_params)
{
  // robot/gazebo/plugin
  const auto plugin = addGazeboPlugin(
    _robot, "tobas_gazebo_ice_propulsion_system_plugin", "tobas::gazebo::GazeboIcePropulsionSystemPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(_ns.c_str());

  // robot/gazebo/plugin/engine
  const auto engine = plugin->InsertNewChildElement("engine");
  engine->InsertNewChildElement("engineConstant")->SetText(toString(_engine_param.engine_const).c_str());
  engine->InsertNewChildElement("timeConstUp")->SetText(toString(_engine_param.time_const_up).c_str());
  engine->InsertNewChildElement("timeConstDown")->SetText(toString(_engine_param.time_const_down).c_str());

  // robot/gazebo/plugin/rotor
  for (const auto& rotor_param : _rotor_params) {
    const auto rotor = plugin->InsertNewChildElement("rotor");
    rotor->InsertNewChildElement("linkName")->SetText(rotor_param.link_name.c_str());
    rotor->InsertNewChildElement("turningDirection")->SetText(textFromEnum(rotor_param.direction).c_str());
    rotor->InsertNewChildElement("gearRatio")->SetText(toString(rotor_param.gear_ratio).c_str());
    rotor->InsertNewChildElement("numberOfBlades")->SetText(toString(rotor_param.num_blades).c_str());
    rotor->InsertNewChildElement("minPitchAngle")->SetText(toString(rotor_param.pitch_angle_limit.lower).c_str());
    rotor->InsertNewChildElement("maxPitchAngle")->SetText(toString(rotor_param.pitch_angle_limit.upper).c_str());
    rotor->InsertNewChildElement("maxPitchAngleRate")->SetText(toString(rotor_param.max_pitch_angle_rate).c_str());

    const auto& ct = rotor_param.motor_const;
    rotor->InsertNewChildElement("motorConstant")->SetText(toString(std::vector<double>{ ct.c0, ct.c1 }).c_str());

    const auto& cm = rotor_param.moment_const;
    rotor->InsertNewChildElement("momentConstant")
      ->SetText(toString(std::vector<double>{ cm.a, cm.b, cm.c, cm.phi0 }).c_str());

    const auto& ch = rotor_param.drag_const;
    rotor->InsertNewChildElement("dragConstant")->SetText(toString(std::vector<double>{ ch.c0, ch.c1 }).c_str());
  }
}

void addFixedWingPlugin(
  tinyxml2::XMLElement* _robot,
  const std::string& _ns,
  const std::string& _base_link_name,
  const FixedWingConfig& _fixed_wing)
{
  const auto& vehicle = _fixed_wing.vehicle;
  const auto& aerodynamics = _fixed_wing.aerodynamics;
  const auto& control_surfaces = _fixed_wing.control_surfaces;

  const auto plugin = addGazeboPlugin(_robot, "tobas_gazebo_fixed_wing_plugin", "tobas::gazebo::GazeboFixedWingPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(_ns.c_str());
  plugin->InsertNewChildElement("baseLinkName")->SetText(_base_link_name.c_str());

  // Vehicle
  plugin->InsertNewChildElement("wingSurface")->SetText(toString(vehicle.wing_surface).c_str());
  plugin->InsertNewChildElement("wingSpan")->SetText(toString(vehicle.wing_span).c_str());
  plugin->InsertNewChildElement("meanAerodynamicChord")->SetText(toString(vehicle.mac).c_str());
  plugin->InsertNewChildElement("aerodynamicCenter")->SetText(toString(vehicle.ac.data).c_str());
  plugin->InsertNewChildElement("lowerStallAngle")->SetText(toString(vehicle.alpha_limit.lower).c_str());
  plugin->InsertNewChildElement("upperStallAngle")->SetText(toString(vehicle.alpha_limit.upper).c_str());

  // Aerodynamic Coefficients
  plugin->InsertNewChildElement("cLift0")->SetText(toString(aerodynamics.c_lift_0).c_str());
  plugin->InsertNewChildElement("cLiftAlpha")->SetText(toString(aerodynamics.c_lift_alpha).c_str());
  plugin->InsertNewChildElement("cDrag0")->SetText(toString(aerodynamics.c_drag_0).c_str());
  plugin->InsertNewChildElement("cDragAlpha")->SetText(toString(aerodynamics.c_drag_alpha).c_str());
  plugin->InsertNewChildElement("cSideBeta")->SetText(toString(aerodynamics.c_side_beta).c_str());
  plugin->InsertNewChildElement("cRollBeta")->SetText(toString(aerodynamics.c_roll_beta).c_str());
  plugin->InsertNewChildElement("cRollP")->SetText(toString(aerodynamics.c_roll_p).c_str());
  plugin->InsertNewChildElement("cRollR")->SetText(toString(aerodynamics.c_roll_r).c_str());
  plugin->InsertNewChildElement("cPitch0")->SetText(toString(aerodynamics.c_pitch_0).c_str());
  plugin->InsertNewChildElement("cPitchAlpha")->SetText(toString(aerodynamics.c_pitch_alpha).c_str());
  plugin->InsertNewChildElement("cPitchAbsBeta")->SetText(toString(aerodynamics.c_pitch_abs_beta).c_str());
  plugin->InsertNewChildElement("cPitchAlphaRate")->SetText(toString(aerodynamics.c_pitch_alpha_rate).c_str());
  plugin->InsertNewChildElement("cPitchQ")->SetText(toString(aerodynamics.c_pitch_q).c_str());
  plugin->InsertNewChildElement("cYawBeta")->SetText(toString(aerodynamics.c_yaw_beta).c_str());
  plugin->InsertNewChildElement("cYawP")->SetText(toString(aerodynamics.c_yaw_p).c_str());
  plugin->InsertNewChildElement("cYawR")->SetText(toString(aerodynamics.c_yaw_r).c_str());

  // Control Surfaces
  for (const auto& [_, cs] : control_surfaces) {
    const auto cs_elem = plugin->InsertNewChildElement("controlSurface");
    cs_elem->InsertNewChildElement("linkName")->SetText(cs.link_name.c_str());
    cs_elem->InsertNewChildElement("cLiftDelta")->SetText(toString(cs.c_lift_delta).c_str());
    cs_elem->InsertNewChildElement("cDragAbsDelta")->SetText(toString(cs.c_drag_abs_delta).c_str());
    cs_elem->InsertNewChildElement("cSideDelta")->SetText(toString(cs.c_side_delta).c_str());
    cs_elem->InsertNewChildElement("cRollDelta")->SetText(toString(cs.c_roll_delta).c_str());
    cs_elem->InsertNewChildElement("cPitchDelta")->SetText(toString(cs.c_pitch_delta).c_str());
    cs_elem->InsertNewChildElement("cYawDelta")->SetText(toString(cs.c_yaw_delta).c_str());
  }
}

void addJointStateBroadcasterPlugin(
  tinyxml2::XMLElement* _robot,
  const std::string& _ns,
  const std::vector<std::string>& _joint_names,
  int _update_rate)
{
  const auto plugin = addGazeboPlugin(
    _robot, "tobas_gazebo_joint_state_broadcaster_plugin", "tobas::gazebo::GazeboJointStateBroadcasterPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(_ns.c_str());
  addList(plugin, "jointNames", _joint_names);
  plugin->InsertNewChildElement("updateRate")->SetText(toString(_update_rate).c_str());
}

void addJointPositionControllerPlugin(
  tinyxml2::XMLElement* _robot,
  const std::string& _ns,
  const std::string& _joint_name,
  double _home_pos,
  double _time_const)
{
  const auto plugin = addGazeboPlugin(
    _robot, "tobas_gazebo_joint_position_controller_plugin", "tobas::gazebo::GazeboJointPositionControllerPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(_ns.c_str());
  plugin->InsertNewChildElement("jointName")->SetText(_joint_name.c_str());
  plugin->InsertNewChildElement("homePosition")->SetText(toString(_home_pos).c_str());
  plugin->InsertNewChildElement("timeConstant")->SetText(toString(_time_const).c_str());
}

void addJointVelocityControllerPlugin(
  tinyxml2::XMLElement* _robot,
  const std::string& _ns,
  const std::string& _joint_name,
  double _home_pos)
{
  const auto plugin = addGazeboPlugin(
    _robot, "tobas_gazebo_joint_velocity_controller_plugin", "tobas::gazebo::GazeboJointVelocityControllerPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(_ns.c_str());
  plugin->InsertNewChildElement("jointName")->SetText(_joint_name.c_str());
  plugin->InsertNewChildElement("homePosition")->SetText(toString(_home_pos).c_str());
}

void addJointEffortControllerPlugin(
  tinyxml2::XMLElement* _robot,
  const std::string& _ns,
  const std::string& _joint_name,
  double _home_pos)
{
  const auto plugin = addGazeboPlugin(
    _robot, "tobas_gazebo_joint_effort_controller_plugin", "tobas::gazebo::GazeboJointEffortControllerPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(_ns.c_str());
  plugin->InsertNewChildElement("jointName")->SetText(_joint_name.c_str());
  plugin->InsertNewChildElement("homePosition")->SetText(toString(_home_pos).c_str());
}

void addGazeboWindPlugin(tinyxml2::XMLElement* _robot, const std::string& _ns, const std::string& _link_name)
{
  const auto plugin = addGazeboPlugin(_robot, "tobas_gazebo_wind_plugin", "tobas::gazebo::GazeboWindPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(_ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(_link_name.c_str());
}

void addGazeboGroundTruthStatePlugin(tinyxml2::XMLElement* _robot, const std::string& _ns, const std::string& _link_name)
{
  const auto plugin =
    addGazeboPlugin(_robot, "tobas_gazebo_ground_truth_state_plugin", "tobas::gazebo::GazeboGroundTruthStatePlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(_ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(_link_name.c_str());
}

void addGazeboLookAtPositionPlugin(tinyxml2::XMLElement* _robot, const std::string& _ns, const std::string& _link_name)
{
  const auto plugin =
    addGazeboPlugin(_robot, "tobas_gazebo_lookat_position_plugin", "tobas::gazebo::GazeboLookAtPositionPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(_ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(_link_name.c_str());
}

void addGazeboFixedLoadPlugin(tinyxml2::XMLElement* _robot, const std::string& _ns, const std::string& _link_name)
{
  const auto plugin = addGazeboPlugin(_robot, "tobas_gazebo_fixed_load_plugin", "tobas::gazebo::GazeboFixedLoadPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(_ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(_link_name.c_str());
}

void addGazeboSuspendedLoadPlugin(tinyxml2::XMLElement* _robot, const std::string& _ns, const std::string& _link_name)
{
  const auto plugin =
    addGazeboPlugin(_robot, "tobas_gazebo_suspended_load_plugin", "tobas::gazebo::GazeboSuspendedLoadPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(_ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(_link_name.c_str());
}

void addBaseStaticJoint(tinyxml2::XMLElement* _robot, const std::string& _root_link_name)
{
  // robot/xacro:if
  const auto xacro_if = _robot->InsertNewChildElement("xacro:if");
  xacro_if->SetAttribute("value", "$(arg DEBUG)");

  // robot/xacro:if/link
  const auto link = xacro_if->InsertNewChildElement("link");
  link->SetAttribute("name", frame::kWorld);

  // robot/xacro:if/joint
  const auto joint = xacro_if->InsertNewChildElement("joint");
  joint->SetAttribute("name", "base_static_joint");
  joint->SetAttribute("type", "fixed");
  joint->InsertNewChildElement("parent")->SetAttribute("link", frame::kWorld);
  joint->InsertNewChildElement("child")->SetAttribute("link", _root_link_name.c_str());
}
}  // namespace tobas::gui::sa::xml

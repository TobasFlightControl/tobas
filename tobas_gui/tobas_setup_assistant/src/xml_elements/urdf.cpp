// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/xml_elements/urdf.hpp"

#include <format>
#include <ranges>

#include <tobas_constants/frame.hpp>
#include <tobas_path_tools/join.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace xml
{
namespace
{
template <typename T>
std::string toString(const T& data)
{
  return std::to_string(data);
}

template <>
std::string toString<std::string>(const std::string& data)
{
  return data;
}

template <>
std::string toString<double>(const double& data)
{
  return std::format("{:.{}g}", data, 9);
}

template <>
std::string toString<std::vector<double>>(const std::vector<double>& data)
{
  std::string res;
  for (const auto& [i, x] : std::views::enumerate(data)) {
    if (i != 0) {
      res += ' ';
    }
    res += toString(x);
  }
  return res;
}

template <>
std::string toString<std::pair<double, double>>(const std::pair<double, double>& data)
{
  // `format("{} {}", first, second)` may cause mojibake, so convert each character to a string.
  return toString(data.first) + ' ' + toString(data.second);
}

template <>
std::string toString<Eigen::Vector3d>(const Eigen::Vector3d& data)
{
  return toString(data.x()) + ' ' + toString(data.y()) + ' ' + toString(data.z());
}

template <typename T>
void addList(tinyxml2::XMLElement* parent, const std::string& list_name, const std::vector<T>& items)
{
  const auto elem = parent->InsertNewChildElement(list_name.c_str());
  for (const auto& item : items) {
    elem->InsertNewChildElement("item")->SetText(toString(item).c_str());
  }
}

tinyxml2::XMLElement* addGazeboPlugin(tinyxml2::XMLElement* robot, const std::string& filename, const std::string& name)
{
  // robot/gazebo
  const auto gazebo = robot->InsertNewChildElement("gazebo");

  // robot/gazebo/plugin
  const auto plugin = gazebo->InsertNewChildElement("plugin");
  plugin->SetAttribute("filename", filename.c_str());
  plugin->SetAttribute("name", name.c_str());

  return plugin;
}
}  // namespace

void addBatteryPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  int update_rate,
  double max_voltage,
  double sag_voltage,
  double max_current,
  double current_capacity,
  double internal_registance,
  const std::vector<std::string>& rotor_link_names)
{
  const auto plugin = addGazeboPlugin(robot, "tobas_gazebo_battery_plugin", "tobas::gazebo::GazeboBatteryPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("updateRate")->SetText(toString(update_rate).c_str());
  plugin->InsertNewChildElement("maxVoltage")->SetText(toString(max_voltage).c_str());
  plugin->InsertNewChildElement("sagVoltage")->SetText(toString(sag_voltage).c_str());
  plugin->InsertNewChildElement("maxCurrent")->SetText(toString(max_current).c_str());
  plugin->InsertNewChildElement("currentCapacity")->SetText(toString(current_capacity).c_str());
  plugin->InsertNewChildElement("internalRegistance")->SetText(toString(internal_registance).c_str());
  addList(plugin, "rotorLinkNames", rotor_link_names);
}

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
  const std::vector<std::string>& rotor_link_names)
{
  const auto plugin = addGazeboPlugin(robot, "tobas_gazebo_imu_plugin", "tobas::gazebo::GazeboImuPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(link_name.c_str());
  plugin->InsertNewChildElement("updateRate")->SetText(toString(update_rate).c_str());
  plugin->InsertNewChildElement("offset")->SetText(toString(offset).c_str());
  plugin->InsertNewChildElement("gyroNoiseDensity")->SetText(toString(gyro_noise_density).c_str());
  plugin->InsertNewChildElement("gyroRandomWalk")->SetText(toString(gyro_random_walk).c_str());
  plugin->InsertNewChildElement("gyroBiasCorrelationTime")->SetText(toString(gyro_bias_corr_time).c_str());
  plugin->InsertNewChildElement("accelNoiseDensity")->SetText(toString(acc_noise_density).c_str());
  plugin->InsertNewChildElement("accelRandomWalk")->SetText(toString(acc_random_walk).c_str());
  plugin->InsertNewChildElement("accelBiasCorrelationTime")->SetText(toString(acc_bias_corr_time).c_str());
  addList(plugin, "rotorLinkNames", rotor_link_names);
}

void addMagnetometerPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& link_name,
  int update_rate,
  const Eigen::Vector3d& offset,
  double noise_stddev,
  double hard_bias_norm)
{
  const auto plugin =
    addGazeboPlugin(robot, "tobas_gazebo_magnetometer_plugin", "tobas::gazebo::GazeboMagnetometerPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(link_name.c_str());
  plugin->InsertNewChildElement("updateRate")->SetText(toString(update_rate).c_str());
  plugin->InsertNewChildElement("offset")->SetText(toString(offset).c_str());
  plugin->InsertNewChildElement("noiseStddev")->SetText(toString(noise_stddev).c_str());
  plugin->InsertNewChildElement("hardBiasNorm")->SetText(toString(hard_bias_norm).c_str());
}

void addBarometerPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& link_name,
  int update_rate,
  const Eigen::Vector3d& offset,
  double noise_stddev)
{
  const auto plugin = addGazeboPlugin(robot, "tobas_gazebo_barometer_plugin", "tobas::gazebo::GazeboBarometerPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(link_name.c_str());
  plugin->InsertNewChildElement("updateRate")->SetText(toString(update_rate).c_str());
  plugin->InsertNewChildElement("offset")->SetText(toString(offset).c_str());
  plugin->InsertNewChildElement("noiseStddev")->SetText(toString(noise_stddev).c_str());
}

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
  double ver_vel_stddev,
  double hor_vel_stddev)
{
  const auto plugin = addGazeboPlugin(robot, "tobas_gazebo_gnss_plugin", "tobas::gazebo::GazeboGnssPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(link_name.c_str());
  plugin->InsertNewChildElement("updateRate")->SetText(toString(update_rate).c_str());
  plugin->InsertNewChildElement("offset")->SetText(toString(offset).c_str());
  plugin->InsertNewChildElement("delay")->SetText(toString(delay).c_str());
  plugin->InsertNewChildElement("positionCorrTime")->SetText(toString(position_corr_time).c_str());
  plugin->InsertNewChildElement("horPosAccuracy")->SetText(toString(hor_pos_accuracy).c_str());
  plugin->InsertNewChildElement("verPosAccuracy")->SetText(toString(ver_pos_accuracy).c_str());
  plugin->InsertNewChildElement("horVelStdDev")->SetText(toString(hor_vel_stddev).c_str());
  plugin->InsertNewChildElement("verVelStdDev")->SetText(toString(ver_vel_stddev).c_str());
}

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
  double max_current)
{
  const auto plugin = addGazeboPlugin(
    robot, "tobas_gazebo_electric_propulsion_system_plugin", "tobas::gazebo::GazeboElectricPropulsionSystemPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(link_name.c_str());
  plugin->InsertNewChildElement("kv")->SetText(toString(kv).c_str());
  plugin->InsertNewChildElement("internalResistance")->SetText(internal_resistance);
  plugin->InsertNewChildElement("numberOfBlades")->SetText(toString(num_blades).c_str());
  plugin->InsertNewChildElement("motorConstant")->SetText(toString(motor_const).c_str());
  plugin->InsertNewChildElement("momentConstant")->SetText(toString(moment_const).c_str());
  plugin->InsertNewChildElement("dragConstant")->SetText(toString(drag_const).c_str());
  plugin->InsertNewChildElement("turningDirection")->SetText(textFromEnum(direction).c_str());
  plugin->InsertNewChildElement("maxCurrent")->SetText(toString(max_current).c_str());
}

void addIcePropulsionSystemPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const EngineParam& engine_param,
  const std::vector<IceRotorParam>& rotor_params)
{
  // robot/gazebo/plugin
  const auto plugin = addGazeboPlugin(
    robot, "tobas_gazebo_ice_propulsion_system_plugin", "tobas::gazebo::GazeboIcePropulsionSystemPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());

  // robot/gazebo/plugin/engine
  const auto engine = plugin->InsertNewChildElement("engine");
  engine->InsertNewChildElement("engineConstant")->SetText(toString(engine_param.engine_const).c_str());
  engine->InsertNewChildElement("timeConstUp")->SetText(toString(engine_param.time_const_up).c_str());
  engine->InsertNewChildElement("timeConstDown")->SetText(toString(engine_param.time_const_down).c_str());

  // robot/gazebo/plugin/rotor
  for (const auto& rotor_param : rotor_params) {
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
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& base_link_name,
  const FixedWingConfig& fixed_wing)
{
  const auto& vehicle = fixed_wing.vehicle;
  const auto& aerodynamics = fixed_wing.aerodynamics;
  const auto& control_surfaces = fixed_wing.control_surfaces;

  const auto plugin = addGazeboPlugin(robot, "tobas_gazebo_fixed_wing_plugin", "tobas::gazebo::GazeboFixedWingPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("baseLinkName")->SetText(base_link_name.c_str());

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
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::vector<std::string>& joint_names,
  int update_rate)
{
  const auto plugin = addGazeboPlugin(
    robot, "tobas_gazebo_joint_state_broadcaster_plugin", "tobas::gazebo::GazeboJointStateBroadcasterPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  addList(plugin, "jointNames", joint_names);
  plugin->InsertNewChildElement("updateRate")->SetText(toString(update_rate).c_str());
}

void addJointPositionControllerPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& joint_name,
  double home_pos,
  double time_const)
{
  const auto plugin = addGazeboPlugin(
    robot, "tobas_gazebo_joint_position_controller_plugin", "tobas::gazebo::GazeboJointPositionControllerPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("jointName")->SetText(joint_name.c_str());
  plugin->InsertNewChildElement("homePosition")->SetText(toString(home_pos).c_str());
  plugin->InsertNewChildElement("timeConstant")->SetText(toString(time_const).c_str());
}

void addJointVelocityControllerPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& joint_name,
  double home_pos)
{
  const auto plugin = addGazeboPlugin(
    robot, "tobas_gazebo_joint_velocity_controller_plugin", "tobas::gazebo::GazeboJointVelocityControllerPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("jointName")->SetText(joint_name.c_str());
  plugin->InsertNewChildElement("homePosition")->SetText(toString(home_pos).c_str());
}

void addJointEffortControllerPlugin(
  tinyxml2::XMLElement* robot,
  const std::string& ns,
  const std::string& joint_name,
  double home_pos)
{
  const auto plugin = addGazeboPlugin(
    robot, "tobas_gazebo_joint_effort_controller_plugin", "tobas::gazebo::GazeboJointEffortControllerPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("jointName")->SetText(joint_name.c_str());
  plugin->InsertNewChildElement("homePosition")->SetText(toString(home_pos).c_str());
}

void addGazeboWindPlugin(tinyxml2::XMLElement* robot, const std::string& ns, const std::string& link_name)
{
  const auto plugin = addGazeboPlugin(robot, "tobas_gazebo_wind_plugin", "tobas::gazebo::GazeboWindPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(link_name.c_str());
}

void addGazeboGroundTruthStatePlugin(tinyxml2::XMLElement* robot, const std::string& ns, const std::string& link_name)
{
  const auto plugin =
    addGazeboPlugin(robot, "tobas_gazebo_ground_truth_state_plugin", "tobas::gazebo::GazeboGroundTruthStatePlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(link_name.c_str());
}

void addGazeboLookAtPositionPlugin(tinyxml2::XMLElement* robot, const std::string& ns, const std::string& link_name)
{
  const auto plugin =
    addGazeboPlugin(robot, "tobas_gazebo_lookat_position_plugin", "tobas::gazebo::GazeboLookAtPositionPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(link_name.c_str());
}

void addGazeboSuspendedLoadPlugin(tinyxml2::XMLElement* robot, const std::string& ns, const std::string& link_name)
{
  const auto plugin =
    addGazeboPlugin(robot, "tobas_gazebo_suspended_load_plugin", "tobas::gazebo::GazeboSuspendedLoadPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(link_name.c_str());
}

void addBaseStaticJoint(tinyxml2::XMLElement* robot, const std::string& root_link_name)
{
  // robot/xacro:if
  const auto xacro_if = robot->InsertNewChildElement("xacro:if");
  xacro_if->SetAttribute("value", "$(arg DEBUG)");

  // robot/xacro:if/link
  const auto link = xacro_if->InsertNewChildElement("link");
  link->SetAttribute("name", frame::kWorld);

  // robot/xacro:if/joint
  const auto joint = xacro_if->InsertNewChildElement("joint");
  joint->SetAttribute("name", "base_static_joint");
  joint->SetAttribute("type", "fixed");
  joint->InsertNewChildElement("parent")->SetAttribute("link", frame::kWorld);
  joint->InsertNewChildElement("child")->SetAttribute("link", root_link_name.c_str());
}
}  // namespace xml
}  // namespace sa
}  // namespace gui
}  // namespace tobas

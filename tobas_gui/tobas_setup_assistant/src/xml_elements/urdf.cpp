#include <format>

#include <tobas_path_tools/join.hpp>
#include <tobas_constants/constants.hpp>

#include "tobas_setup_assistant/xml_elements/urdf.hpp"

using namespace std;

namespace gui
{
namespace setup_assistant
{
namespace util
{
string toString(double data)
{
  return format("{}", data);  // 最適な表記方法を自動判定
}

string toString(const pair<double, double>& data)
{
  // format("{} {}", double, double) だと文字化けする可能性があるため，1文字ずつ文字列に変換する．
  return toString(data.first) + " " + toString(data.second);
}

string toString(const Eigen::Vector3d& data)
{
  return toString(data.x()) + " " + toString(data.y()) + " " + toString(data.z());
}

template <typename T>
void addList(tinyxml2::XMLElement* parent, const string& list_name, const vector<T>& items)
{
  const auto elem = parent->InsertNewChildElement(list_name.c_str());
  for (const auto& item : items)
    elem->InsertNewChildElement("item")->SetText(to_string(item).c_str());
}

tinyxml2::XMLElement* addGazeboPlugin(tinyxml2::XMLElement* robot, const string& filename, const string& name)
{
  // robot/gazebo
  const auto gazebo = robot->InsertNewChildElement("gazebo");

  // robot/gazebo/plugin
  const auto plugin = gazebo->InsertNewChildElement("plugin");
  plugin->SetAttribute("filename", filename.c_str());
  plugin->SetAttribute("name", name.c_str());

  return plugin;
}

tinyxml2::XMLElement*
addROS2ControlStateIF(tinyxml2::XMLElement* joint, tobas::jnt_cmd_iface_t interface, double init_value = 0.)
{
  const auto state_if_elem = joint->InsertNewChildElement("state_interface");
  state_if_elem->SetAttribute("name", tobas::jntCmdIfaceEnumToText(interface).c_str());

  const auto init_value_elem = state_if_elem->InsertNewChildElement("param");
  init_value_elem->SetAttribute("name", "initial_value");
  init_value_elem->SetText(init_value);

  return state_if_elem;
}

tinyxml2::XMLElement* addROS2ControlCommandIF(tinyxml2::XMLElement* joint, tobas::jnt_cmd_iface_t interface)
{
  const auto command_if_elem = joint->InsertNewChildElement("command_interface");
  command_if_elem->SetAttribute("name", tobas::jntCmdIfaceEnumToText(interface).c_str());
  return command_if_elem;
}
}  // namespace util

void addBatteryPlugin(
  tinyxml2::XMLElement* robot,
  const string& ns,
  double update_rate,
  double max_voltage,
  double sag_voltage,
  double max_current,
  double current_capacity,
  double internal_registance,
  const vector<size_t>& rotor_channels)
{
  const auto plugin = util::addGazeboPlugin(robot, "tobas_gazebo_battery_plugin", "gazebo::GazeboBatteryPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("updateRate")->SetText(update_rate);
  plugin->InsertNewChildElement("maxVoltage")->SetText(max_voltage);
  plugin->InsertNewChildElement("sagVoltage")->SetText(sag_voltage);
  plugin->InsertNewChildElement("maxCurrent")->SetText(max_current);
  plugin->InsertNewChildElement("currentCapacity")->SetText(current_capacity);
  plugin->InsertNewChildElement("internalRegistance")->SetText(internal_registance);
  util::addList(plugin, "rotorChannels", rotor_channels);
}

void addIMUPlugin(
  tinyxml2::XMLElement* robot,
  const string& ns,
  const string& link_name,
  double update_rate,
  const Eigen::Vector3d& offset,
  double gyro_noise_density,
  double gyro_random_walk,
  double gyro_bias_corr_time,
  double gyro_turn_on_bias_sigma,
  double acc_noise_density,
  double acc_random_walk,
  double acc_bias_corr_time,
  double acc_turn_on_bias_sigma,
  const vector<size_t>& rotor_channels)
{
  const auto plugin = util::addGazeboPlugin(robot, "tobas_gazebo_imu_plugin", "gazebo::GazeboImuPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(link_name.c_str());
  plugin->InsertNewChildElement("updateRate")->SetText(update_rate);
  plugin->InsertNewChildElement("offset")->SetText(util::toString(offset).c_str());
  plugin->InsertNewChildElement("gyroNoiseDensityOnSignal")->SetText(gyro_noise_density);
  plugin->InsertNewChildElement("gyroNoiseDensityObserved")->SetText(gyro_noise_density);
  plugin->InsertNewChildElement("gyroRandomWalk")->SetText(gyro_random_walk);
  plugin->InsertNewChildElement("gyroBiasCorrelationTime")->SetText(gyro_bias_corr_time);
  plugin->InsertNewChildElement("gyroTurnOnBiasSigma")->SetText(gyro_turn_on_bias_sigma);
  plugin->InsertNewChildElement("accelNoiseDensityOnSignal")->SetText(acc_noise_density);
  plugin->InsertNewChildElement("accelNoiseDensityObserved")->SetText(acc_noise_density);
  plugin->InsertNewChildElement("accelRandomWalk")->SetText(acc_random_walk);
  plugin->InsertNewChildElement("accelBiasCorrelationTime")->SetText(acc_bias_corr_time);
  plugin->InsertNewChildElement("accelTurnOnBiasSigma")->SetText(acc_turn_on_bias_sigma);
  util::addList(plugin, "rotorChannels", rotor_channels);
}

void addMagnetometerPlugin(
  tinyxml2::XMLElement* robot,
  const string& ns,
  const string& link_name,
  double update_rate,
  const Eigen::Vector3d& offset,
  double latitude_zero,
  double longitude_zero,
  double altitude_zero,
  double noise_stddev,
  double hard_bias_range)
{
  const auto plugin =
    util::addGazeboPlugin(robot, "tobas_gazebo_magnetometer_plugin", "gazebo::GazeboMagnetometerPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(link_name.c_str());
  plugin->InsertNewChildElement("updateRate")->SetText(update_rate);
  plugin->InsertNewChildElement("offset")->SetText(util::toString(offset).c_str());
  plugin->InsertNewChildElement("latitudeZero")->SetText(latitude_zero);
  plugin->InsertNewChildElement("longitudeZero")->SetText(longitude_zero);
  plugin->InsertNewChildElement("altitudeZero")->SetText(altitude_zero);
  plugin->InsertNewChildElement("noiseStddev")->SetText(noise_stddev);
  plugin->InsertNewChildElement("hardBiasRange")->SetText(hard_bias_range);
}

void addBarometerPlugin(
  tinyxml2::XMLElement* robot,
  const string& ns,
  const string& link_name,
  double update_rate,
  const Eigen::Vector3d& offset,
  double altitude_zero,
  double pressure_variance)
{
  const auto plugin = util::addGazeboPlugin(robot, "tobas_gazebo_barometer_plugin", "gazebo::GazeboBarometerPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(link_name.c_str());
  plugin->InsertNewChildElement("updateRate")->SetText(update_rate);
  plugin->InsertNewChildElement("offset")->SetText(util::toString(offset).c_str());
  plugin->InsertNewChildElement("altitudeZero")->SetText(altitude_zero);
  plugin->InsertNewChildElement("pressureVariance")->SetText(pressure_variance);
}

void addGNSSPlugin(
  tinyxml2::XMLElement* robot,
  const string& ns,
  const string& link_name,
  double update_rate,
  const Eigen::Vector3d& offset,
  double delay,
  double position_corr_time,
  double hor_pos_accuracy,
  double ver_pos_accuracy,
  double ver_vel_stddev,
  double hor_vel_stddev,
  double latitude_zero,
  double longitude_zero,
  double altitude_zero)
{
  const auto plugin = util::addGazeboPlugin(robot, "tobas_gazebo_gnss_plugin", "gazebo::GazeboGnssPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(link_name.c_str());
  plugin->InsertNewChildElement("updateRate")->SetText(update_rate);
  plugin->InsertNewChildElement("offset")->SetText(util::toString(offset).c_str());
  plugin->InsertNewChildElement("delay")->SetText(delay);
  plugin->InsertNewChildElement("positionCorrTime")->SetText(position_corr_time);
  plugin->InsertNewChildElement("horPosAccuracy")->SetText(hor_pos_accuracy);
  plugin->InsertNewChildElement("verPosAccuracy")->SetText(ver_pos_accuracy);
  plugin->InsertNewChildElement("horVelStdDev")->SetText(hor_vel_stddev);
  plugin->InsertNewChildElement("verVelStdDev")->SetText(ver_vel_stddev);
  plugin->InsertNewChildElement("latitudeZero")->SetText(latitude_zero);
  plugin->InsertNewChildElement("longitudeZero")->SetText(longitude_zero);
  plugin->InsertNewChildElement("altitudeZero")->SetText(altitude_zero);
}

void addRotorPlugin(
  tinyxml2::XMLElement* robot,
  const string& ns,
  const string& link_name,
  uint32_t channel,
  double kv,
  double internal_resistance,
  size_t num_blades,
  double motor_constant,
  double moment_constant,
  double drag_constant,
  tobas::turning_direction_t direction,
  double max_current,
  double max_model_error_rate)
{
  const auto plugin = util::addGazeboPlugin(robot, "tobas_gazebo_rotor_plugin", "gazebo::GazeboRotorPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(link_name.c_str());
  plugin->InsertNewChildElement("channel")->SetText(channel);
  plugin->InsertNewChildElement("kv")->SetText(kv);
  plugin->InsertNewChildElement("internalResistance")->SetText(internal_resistance);
  plugin->InsertNewChildElement("numberOfBlades")->SetText(num_blades);
  plugin->InsertNewChildElement("motorConstant")->SetText(motor_constant);
  plugin->InsertNewChildElement("momentConstant")->SetText(moment_constant);
  plugin->InsertNewChildElement("rotorDragCoefficient")->SetText(drag_constant);
  plugin->InsertNewChildElement("turningDirection")->SetText(direction);
  plugin->InsertNewChildElement("maxCurrent")->SetText(max_current);
  plugin->InsertNewChildElement("maxModelErrorRate")->SetText(max_model_error_rate);
}

void addFixedWingPlugin(
  tinyxml2::XMLElement* robot,
  const string& ns,
  const string& link_name,
  double altitude_zero,
  const tobas::FixedWingConfig& fixed_wing)
{
  const auto& vehicle = fixed_wing.vehicle;
  const auto& aerodynamics = fixed_wing.aerodynamics;
  const auto& control_surfaces = fixed_wing.control_surfaces;

  const auto plugin = util::addGazeboPlugin(robot, "tobas_gazebo_fixed_wing_plugin", "gazebo::GazeboFixedWingPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(link_name.c_str());
  plugin->InsertNewChildElement("altitudeZero")->SetText(altitude_zero);

  // Vehicle
  plugin->InsertNewChildElement("wingSurface")->SetText(vehicle.wing_surface);
  plugin->InsertNewChildElement("wingSpan")->SetText(vehicle.wing_span);
  plugin->InsertNewChildElement("meanAerodynamicChord")->SetText(vehicle.mac);
  plugin->InsertNewChildElement("aerodynamicCenter")->SetText(util::toString(vehicle.ac.data).c_str());
  plugin->InsertNewChildElement("lowerStallAngle")->SetText(vehicle.alpha_limit.lower);
  plugin->InsertNewChildElement("upperStallAngle")->SetText(vehicle.alpha_limit.upper);

  // Aerodynamic Coefficients
  plugin->InsertNewChildElement("cLift0")->SetText(aerodynamics.c_lift_0);
  plugin->InsertNewChildElement("cLiftAlpha")->SetText(aerodynamics.c_lift_alpha);
  plugin->InsertNewChildElement("cDrag0")->SetText(aerodynamics.c_drag_0);
  plugin->InsertNewChildElement("cDragAlpha")->SetText(aerodynamics.c_drag_alpha);
  plugin->InsertNewChildElement("cSideBeta")->SetText(aerodynamics.c_side_beta);
  plugin->InsertNewChildElement("cRollBeta")->SetText(aerodynamics.c_roll_beta);
  plugin->InsertNewChildElement("cRollP")->SetText(aerodynamics.c_roll_p);
  plugin->InsertNewChildElement("cRollR")->SetText(aerodynamics.c_roll_r);
  plugin->InsertNewChildElement("cPitch0")->SetText(aerodynamics.c_pitch_0);
  plugin->InsertNewChildElement("cPitchAlpha")->SetText(aerodynamics.c_pitch_alpha);
  plugin->InsertNewChildElement("cPitchAbsBeta")->SetText(aerodynamics.c_pitch_abs_beta);
  plugin->InsertNewChildElement("cPitchAlphaRate")->SetText(aerodynamics.c_pitch_alpha_rate);
  plugin->InsertNewChildElement("cPitchQ")->SetText(aerodynamics.c_pitch_q);
  plugin->InsertNewChildElement("cYawBeta")->SetText(aerodynamics.c_yaw_beta);
  plugin->InsertNewChildElement("cYawP")->SetText(aerodynamics.c_yaw_p);
  plugin->InsertNewChildElement("cYawR")->SetText(aerodynamics.c_yaw_r);

  // Control Surfaces
  for (const auto& [_, cs] : control_surfaces)
  {
    const auto cs_elem = plugin->InsertNewChildElement("controlSurface");
    cs_elem->InsertNewChildElement("channel")->SetText(cs.channel);
    cs_elem->InsertNewChildElement("linkName")->SetText(cs.link_name.c_str());
    cs_elem->InsertNewChildElement("cLiftDelta")->SetText(cs.c_lift_delta);
    cs_elem->InsertNewChildElement("cDragAbsDelta")->SetText(cs.c_drag_abs_delta);
    cs_elem->InsertNewChildElement("cSideDelta")->SetText(cs.c_side_delta);
    cs_elem->InsertNewChildElement("cRollDelta")->SetText(cs.c_roll_delta);
    cs_elem->InsertNewChildElement("cPitchDelta")->SetText(cs.c_pitch_delta);
    cs_elem->InsertNewChildElement("cYawDelta")->SetText(cs.c_yaw_delta);
  }
}

void addGazeboWindPlugin(tinyxml2::XMLElement* robot, const string& ns, const string& link_name)
{
  const auto plugin = util::addGazeboPlugin(robot, "tobas_gazebo_wind_plugin", "gazebo::GazeboWindPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(link_name.c_str());
}

void addGazeboGroundTruthStatePlugin(tinyxml2::XMLElement* robot, const string& ns, const string& link_name)
{
  const auto plugin =
    util::addGazeboPlugin(robot, "tobas_gazebo_ground_truth_state_plugin", "gazebo::GazeboGroundTruthStatePlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(link_name.c_str());
}

void addGazeboROS2SimSystem(tinyxml2::XMLElement* robot, const tobas::JointConfigMap& joints)
{
  // robot/ros2_control
  const auto ros2_control = robot->InsertNewChildElement("ros2_control");
  ros2_control->SetAttribute("name", "GazeboSimSystem");
  ros2_control->SetAttribute("type", "system");

  // robot/ros2_control/hardware
  const auto hardware = ros2_control->InsertNewChildElement("hardware");
  hardware->InsertNewChildElement("plugin")->SetText("gz_ros2_control/GazeboSimSystem");

  // robot/ros2_control/joint
  for (const auto& [jnt_name, jnt_cfg] : joints)
  {
    if (!jnt_cfg.isServoJoint())
      continue;

    const auto joint = ros2_control->InsertNewChildElement("joint");
    joint->SetAttribute("name", jnt_name.c_str());

    // robot/ros2_control/joint/state_interface
    util::addROS2ControlStateIF(joint, tobas::jnt_cmd_iface_t::POSITION, jnt_cfg.home_pos);
    util::addROS2ControlStateIF(joint, tobas::jnt_cmd_iface_t::VELOCITY);
    util::addROS2ControlStateIF(joint, tobas::jnt_cmd_iface_t::EFFORT);

    // robot/ros2_control/joint/command_interface
    util::addROS2ControlCommandIF(joint, jnt_cfg.cmd_iface);
  }
}

void addGazeboSimROS2ControlPlugin(
  tinyxml2::XMLElement* robot,
  const string& ns,
  const string& pkg_name,
  const string& params_rel_path)
{
  const auto plugin =
    util::addGazeboPlugin(robot, "gz_ros2_control-system", "gz_ros2_control::GazeboSimROS2ControlPlugin");

  const auto params = path::join("$(find " + pkg_name + ")", params_rel_path);
  plugin->InsertNewChildElement("parameters")->SetText(params.c_str());

  // robot/gazebo/plugin/ros
  const auto ros = plugin->InsertNewChildElement("ros");
  ros->InsertNewChildElement("namespace")->SetText(ns.c_str());
}

void addBaseStaticJoint(tinyxml2::XMLElement* robot, const string& root_link_name)
{
  // robot/xacro:if
  const auto xacro_if = robot->InsertNewChildElement("xacro:if");
  xacro_if->SetAttribute("value", "$(arg DEBUG)");

  // robot/xacro:if/link
  const auto link = xacro_if->InsertNewChildElement("link");
  link->SetAttribute("name", tobas::kWorldFrame);

  // robot/xacro:if/joint
  const auto joint = xacro_if->InsertNewChildElement("joint");
  joint->SetAttribute("name", "base_static_joint");
  joint->SetAttribute("type", "fixed");
  joint->InsertNewChildElement("parent")->SetAttribute("link", tobas::kWorldFrame);
  joint->InsertNewChildElement("child")->SetAttribute("link", root_link_name.c_str());
}
}  // namespace setup_assistant
}  // namespace gui

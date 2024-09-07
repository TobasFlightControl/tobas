#include <format>

#include <tobas_std_tools/check.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_constants/constants.hpp>

#include "tobas_setup_assistant/xml_nodes.hpp"

using namespace std;

namespace gui
{
namespace setup_assistant
{
namespace util
{
const char* format(const pair<double, double>& data)
{
  return std::format("{} {}", data.first, data.second).c_str();
}

const char* format(const Eigen::Vector3d& data)
{
  return std::format("{} {} {}", data.x(), data.y(), data.z()).c_str();
}

void addList(tinyxml2::XMLElement* parent, const string& list_name, const vector<string>& items)
{
  const auto elem = parent->InsertNewChildElement(list_name.c_str());
  for (const auto& item : items)
    elem->InsertNewChildElement("item")->SetText(item.c_str());
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
  int num_rotors)
{
  TOBAS_CHECK(max_voltage > 0.);
  TOBAS_CHECK(sag_voltage > 0.);
  TOBAS_CHECK(max_current > 0.);
  TOBAS_CHECK(current_capacity > 0.);
  TOBAS_CHECK(internal_registance > 0.);
  TOBAS_CHECK(num_rotors >= 0);

  const auto plugin = util::addGazeboPlugin(robot, "tobas_gazebo_battery_plugin", "gazebo::GazeboBatteryPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("updateRate")->SetText(update_rate);
  plugin->InsertNewChildElement("maxVoltage")->SetText(max_voltage);
  plugin->InsertNewChildElement("sagVoltage")->SetText(sag_voltage);
  plugin->InsertNewChildElement("maxCurrent")->SetText(max_current);
  plugin->InsertNewChildElement("currentCapacity")->SetText(current_capacity);
  plugin->InsertNewChildElement("internalRegistance")->SetText(internal_registance);
  plugin->InsertNewChildElement("numRotors")->SetText(num_rotors);
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
  double gyro_lpf_cutoff_freq,
  double acc_noise_density,
  double acc_random_walk,
  double acc_bias_corr_time,
  double acc_turn_on_bias_sigma,
  double acc_lpf_cutoff_freq)
{
  TOBAS_CHECK(update_rate > 0.);
  TOBAS_CHECK(gyro_noise_density > 0.);
  TOBAS_CHECK(gyro_random_walk > 0.);
  TOBAS_CHECK(gyro_bias_corr_time > 0.);
  TOBAS_CHECK(gyro_turn_on_bias_sigma > 0.);
  TOBAS_CHECK(gyro_lpf_cutoff_freq > 0.);
  TOBAS_CHECK(acc_noise_density > 0.);
  TOBAS_CHECK(acc_random_walk > 0.);
  TOBAS_CHECK(acc_bias_corr_time > 0.);
  TOBAS_CHECK(acc_turn_on_bias_sigma > 0.);
  TOBAS_CHECK(acc_lpf_cutoff_freq > 0.);

  const auto plugin = util::addGazeboPlugin(robot, "tobas_gazebo_imu_plugin", "gazebo::GazeboImuPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(link_name.c_str());
  plugin->InsertNewChildElement("updateRate")->SetText(update_rate);
  plugin->InsertNewChildElement("offset")->SetText(util::format(offset));
  plugin->InsertNewChildElement("gyroNoiseDensityOnSignal")->SetText(gyro_noise_density);
  plugin->InsertNewChildElement("gyroNoiseDensityObserved")->SetText(gyro_noise_density);
  plugin->InsertNewChildElement("gyroRandomWalk")->SetText(gyro_random_walk);
  plugin->InsertNewChildElement("gyroBiasCorrelationTime")->SetText(gyro_bias_corr_time);
  plugin->InsertNewChildElement("gyroTurnOnBiasSigma")->SetText(gyro_turn_on_bias_sigma);
  plugin->InsertNewChildElement("gyroLpfCutoffFreq")->SetText(gyro_lpf_cutoff_freq);
  plugin->InsertNewChildElement("accelNoiseDensityOnSignal")->SetText(acc_noise_density);
  plugin->InsertNewChildElement("accelNoiseDensityObserved")->SetText(acc_noise_density);
  plugin->InsertNewChildElement("accelRandomWalk")->SetText(acc_random_walk);
  plugin->InsertNewChildElement("accelBiasCorrelationTime")->SetText(acc_bias_corr_time);
  plugin->InsertNewChildElement("accelTurnOnBiasSigma")->SetText(acc_turn_on_bias_sigma);
  plugin->InsertNewChildElement("accelLpfCutoffFreq")->SetText(acc_lpf_cutoff_freq);
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
  double gauss_noise,
  double uniform_noise)
{
  TOBAS_CHECK(update_rate > 0.);
  TOBAS_CHECK(gauss_noise > 0.);
  TOBAS_CHECK(uniform_noise > 0.);

  const auto plugin =
    util::addGazeboPlugin(robot, "tobas_gazebo_magnetometer_plugin", "gazebo::GazeboMagnetometerPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(link_name.c_str());
  plugin->InsertNewChildElement("updateRate")->SetText(update_rate);
  plugin->InsertNewChildElement("offset")->SetText(util::format(offset));
  plugin->InsertNewChildElement("latitudeZero")->SetText(latitude_zero);
  plugin->InsertNewChildElement("longitudeZero")->SetText(longitude_zero);
  plugin->InsertNewChildElement("altitudeZero")->SetText(altitude_zero);
  plugin->InsertNewChildElement("noiseNormal")->SetText(gauss_noise);
  plugin->InsertNewChildElement("noiseUniformInitialBias")->SetText(uniform_noise);
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
  TOBAS_CHECK(update_rate > 0.);
  TOBAS_CHECK(altitude_zero > 0.);
  TOBAS_CHECK(pressure_variance > 0.);

  const auto plugin = util::addGazeboPlugin(robot, "tobas_gazebo_barometer_plugin", "gazebo::GazeboBarometerPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(link_name.c_str());
  plugin->InsertNewChildElement("updateRate")->SetText(update_rate);
  plugin->InsertNewChildElement("offset")->SetText(util::format(offset));
  plugin->InsertNewChildElement("altitudeZero")->SetText(altitude_zero);
  plugin->InsertNewChildElement("pressureVariance")->SetText(pressure_variance);
}

void addGPSPlugin(
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
  TOBAS_CHECK(update_rate > 0.);
  TOBAS_CHECK(delay >= 0.);
  TOBAS_CHECK(position_corr_time > 0.);
  TOBAS_CHECK(hor_pos_accuracy > 0.);
  TOBAS_CHECK(ver_pos_accuracy > 0.);
  TOBAS_CHECK(hor_vel_stddev > 0.);
  TOBAS_CHECK(ver_vel_stddev > 0.);
  TOBAS_CHECK(abs(latitude_zero) <= 90.);
  TOBAS_CHECK(abs(longitude_zero) <= 180.);
  TOBAS_CHECK(altitude_zero >= 0.);

  const auto plugin = util::addGazeboPlugin(robot, "tobas_gazebo_gps_plugin", "gazebo::GazeboGpsPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(link_name.c_str());
  plugin->InsertNewChildElement("updateRate")->SetText(update_rate);
  plugin->InsertNewChildElement("offset")->SetText(util::format(offset));
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
  const string& joint_name,
  const tobas::RotorConfig& rotor,
  double time_const_up,
  double time_const_down,
  double max_current,
  double max_model_error_rate)
{
  TOBAS_CHECK(rotor.rot_speed_coefs.first >= 0. && rotor.rot_speed_coefs.second >= 0.);
  TOBAS_CHECK(rotor.motor_constant >= 0.);
  TOBAS_CHECK(rotor.moment_constant >= 0.);
  TOBAS_CHECK(rotor.drag_constant >= 0.);
  TOBAS_CHECK(time_const_up > 0.);
  TOBAS_CHECK(time_const_down > 0.);
  TOBAS_CHECK(rotor.max_rot_speed > 0.);
  TOBAS_CHECK(rotor.num_poles > 0 && rotor.num_poles % 2 == 0);
  TOBAS_CHECK(max_current > 0.);
  TOBAS_CHECK(max_model_error_rate >= 0.);

  const auto plugin = util::addGazeboPlugin(robot, "tobas_gazebo_rotor_plugin", "gazebo::GazeboRotorPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("channel")->SetText(rotor.channel);
  plugin->InsertNewChildElement("jointName")->SetText(joint_name.c_str());
  plugin->InsertNewChildElement("rotSpeedCoefficients")->SetText(util::format(rotor.rot_speed_coefs));
  plugin->InsertNewChildElement("motorConstant")->SetText(rotor.motor_constant);
  plugin->InsertNewChildElement("momentConstant")->SetText(rotor.moment_constant);
  plugin->InsertNewChildElement("rotorDragCoefficient")->SetText(rotor.drag_constant);
  plugin->InsertNewChildElement("turningDirection")->SetText(rotor.direction);
  plugin->InsertNewChildElement("timeConstantUp")->SetText(time_const_up);
  plugin->InsertNewChildElement("timeConstantDown")->SetText(time_const_down);
  plugin->InsertNewChildElement("maxRotationSpeed")->SetText(rotor.max_rot_speed);
  plugin->InsertNewChildElement("numPoles")->SetText(rotor.num_poles);
  plugin->InsertNewChildElement("maxCurrent")->SetText(max_current);
  plugin->InsertNewChildElement("escMode")->SetText(rotor.esc_mode);
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

  TOBAS_CHECK(vehicle.wing_surface > 0.);
  TOBAS_CHECK(vehicle.wing_span > 0.);
  TOBAS_CHECK(vehicle.alpha_limit.inRange(0.));

  const auto plugin = util::addGazeboPlugin(robot, "tobas_gazebo_fixed_wing_plugin", "gazebo::GazeboFixedWingPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  plugin->InsertNewChildElement("linkName")->SetText(link_name.c_str());
  plugin->InsertNewChildElement("altitudeZero")->SetText(altitude_zero);

  // Vehicle
  plugin->InsertNewChildElement("wingSurface")->SetText(vehicle.wing_surface);
  plugin->InsertNewChildElement("wingSpan")->SetText(vehicle.wing_span);
  plugin->InsertNewChildElement("meanAerodynamicChord")->SetText(vehicle.mac);
  plugin->InsertNewChildElement("aerodynamicCenter")->SetText(util::format(vehicle.ac.data));
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
  for (const auto& cs : control_surfaces)
  {
    const auto cs_elem = plugin->InsertNewChildElement("controlSurface");
    cs_elem->InsertNewChildElement("index")->SetText(cs.channel);
    cs_elem->InsertNewChildElement("jointName")->SetText(cs.joint_name.c_str());
    cs_elem->InsertNewChildElement("minAngle")->SetText(cs.angle_limit.lower);
    cs_elem->InsertNewChildElement("maxAngle")->SetText(cs.angle_limit.upper);
    cs_elem->InsertNewChildElement("maxAngleRate")->SetText(cs.max_angle_rate);
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

void addRotorSpeedsPublisherPlugin(
  tinyxml2::XMLElement* robot,
  const string& ns,
  const vector<string>& rotor_joint_names)
{
  const auto plugin = util::addGazeboPlugin(
    robot, "tobas_gazebo_rotor_speeds_publisher_plugin", "gazebo::GazeboRotorSpeedsPublisherPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns.c_str());
  util::addList(plugin, "rotorJointNames", rotor_joint_names);
}

void addGazeboROS2SimSystem()
{
  // TODO: Custom Jointに登録された全てのジョイントのcommand_interfaceとstate_interface
  // cf. https://github.com/ros-controls/gz_ros2_control/tree/rolling/gz_ros2_control_demos/urdf

  // TODO: 既に存在するならば消して上書き
}

void addGazeboSimROS2ControlPlugin(
  tinyxml2::XMLElement* robot,
  const string& ns,
  const string& pkg_name,
  const string& params_rel_path)
{
  // TODO: 既に存在するならば消して上書き

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
  TOBAS_CHECK(root_link_name != tobas::kWorldFrame);

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
  joint->InsertNewChildElement("parent")->SetText(tobas::kWorldFrame);
  joint->InsertNewChildElement("child")->SetText(root_link_name.c_str());
}
}  // namespace setup_assistant
}  // namespace gui

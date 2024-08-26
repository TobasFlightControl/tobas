#include <format>

#include <tobas_std_tools/check.hpp>

#include "tobas_setup_assistant/xml_nodes.hpp"

namespace gui
{
namespace setup_assistant
{
namespace util
{
const char* format(const std::pair<double, double>& data)
{
  return std::format("{} {}", data.first, data.second).c_str();
}

const char* format(const Eigen::Vector3d& data)
{
  return std::format("{} {} {}", data.x(), data.y(), data.z()).c_str();
}

void addList(tinyxml2::XMLElement* parent, const char* list_name, const QStringList& items)
{
  const auto elem = parent->InsertNewChildElement(list_name);
  for (const auto& item : items)
    elem->InsertNewChildElement("item")->SetText(item.toUtf8().constData());
}

tinyxml2::XMLElement* addGazeboPlugin(tinyxml2::XMLElement* robot, const char* filename, const char* name)
{
  // robot/gazebo
  const auto gazebo = robot->InsertNewChildElement("gazebo");

  // robot/gazebo/plugin
  const auto plugin = gazebo->InsertNewChildElement("plugin");
  plugin->SetAttribute("filename", filename);
  plugin->SetAttribute("name", name);

  return plugin;
}
}  // namespace util

void addBatteryPlugin(
  tinyxml2::XMLElement* robot,
  const char* ns,
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
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns);
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
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns);
  plugin->InsertNewChildElement("linkName")->SetText(link_name);
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
  const char* ns,
  const char* link_name,
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
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns);
  plugin->InsertNewChildElement("linkName")->SetText(link_name);
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
  const char* ns,
  const char* link_name,
  double update_rate,
  const Eigen::Vector3d& offset,
  double altitude_zero,
  double pressure_variance)
{
  TOBAS_CHECK(update_rate > 0.);
  TOBAS_CHECK(altitude_zero > 0.);
  TOBAS_CHECK(pressure_variance > 0.);

  const auto plugin = util::addGazeboPlugin(robot, "tobas_gazebo_barometer_plugin", "gazebo::GazeboBarometerPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns);
  plugin->InsertNewChildElement("linkName")->SetText(link_name);
  plugin->InsertNewChildElement("updateRate")->SetText(update_rate);
  plugin->InsertNewChildElement("offset")->SetText(util::format(offset));
  plugin->InsertNewChildElement("altitudeZero")->SetText(altitude_zero);
  plugin->InsertNewChildElement("pressureVariance")->SetText(pressure_variance);
}

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
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns);
  plugin->InsertNewChildElement("linkName")->SetText(link_name);
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
  double max_model_error_rate)
{
  TOBAS_CHECK(rot_speed_coefs.first >= 0. and rot_speed_coefs.second >= 0.);
  TOBAS_CHECK(motor_const >= 0.);
  TOBAS_CHECK(moment_const >= 0.);
  TOBAS_CHECK(rotor_drag_coef >= 0.);
  TOBAS_CHECK(time_const_up > 0.);
  TOBAS_CHECK(time_const_down > 0.);
  TOBAS_CHECK(max_rot_speed > 0.);
  TOBAS_CHECK(num_poles > 0 and num_poles % 2 == 0);
  TOBAS_CHECK(max_current > 0.);
  TOBAS_CHECK(max_model_error_rate >= 0.);

  const auto plugin = util::addGazeboPlugin(robot, "tobas_gazebo_rotor_plugin", "gazebo::GazeboRotorPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns);
  plugin->InsertNewChildElement("channel")->SetText(channel);
  plugin->InsertNewChildElement("jointName")->SetText(joint_name);
  plugin->InsertNewChildElement("rotSpeedCoefficients")->SetText(util::format(rot_speed_coefs));
  plugin->InsertNewChildElement("motorConstant")->SetText(motor_const);
  plugin->InsertNewChildElement("momentConstant")->SetText(moment_const);
  plugin->InsertNewChildElement("rotorDragCoefficient")->SetText(rotor_drag_coef);
  plugin->InsertNewChildElement("turningDirection")->SetText(turning_direction);
  plugin->InsertNewChildElement("timeConstantUp")->SetText(time_const_up);
  plugin->InsertNewChildElement("timeConstantDown")->SetText(time_const_down);
  plugin->InsertNewChildElement("maxRotationSpeed")->SetText(max_rot_speed);
  plugin->InsertNewChildElement("numPoles")->SetText(num_poles);
  plugin->InsertNewChildElement("maxCurrent")->SetText(max_current);
  plugin->InsertNewChildElement("escMode")->SetText(esc_mode);
  plugin->InsertNewChildElement("maxModelErrorRate")->SetText(max_model_error_rate);
}

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
  const tobas::ControlSurfaces& control_surfaces)
{
  TOBAS_CHECK(wing_surface > 0.);
  TOBAS_CHECK(wing_span > 0.);
  TOBAS_CHECK(alpha_limit.first < 0. && 0. < alpha_limit.second);

  const auto plugin = util::addGazeboPlugin(robot, "tobas_gazebo_fixed_wing_plugin", "gazebo::GazeboFixedWingPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns);
  plugin->InsertNewChildElement("linkName")->SetText(link_name);
  plugin->InsertNewChildElement("altitudeZero")->SetText(altitude_zero);

  // Vehicle
  plugin->InsertNewChildElement("wingSurface")->SetText(wing_surface);
  plugin->InsertNewChildElement("wingSpan")->SetText(wing_span);
  plugin->InsertNewChildElement("meanAerodynamicChord")->SetText(mean_aerodynamic_chord);
  plugin->InsertNewChildElement("aerodynamicCenter")->SetText(util::format(aerodynamic_center));
  plugin->InsertNewChildElement("lowerStallAngle")->SetText(alpha_limit.first);
  plugin->InsertNewChildElement("upperStallAngle")->SetText(alpha_limit.second);

  // Aerodynamic Coefficients
  plugin->InsertNewChildElement("cLift0")->SetText(c_lift_0);
  plugin->InsertNewChildElement("cLiftAlpha")->SetText(c_lift_alpha);
  plugin->InsertNewChildElement("cDrag0")->SetText(c_drag_0);
  plugin->InsertNewChildElement("cDragAlpha")->SetText(c_drag_alpha);
  plugin->InsertNewChildElement("cSideBeta")->SetText(c_side_beta);
  plugin->InsertNewChildElement("cRollBeta")->SetText(c_roll_beta);
  plugin->InsertNewChildElement("cRollP")->SetText(c_roll_p);
  plugin->InsertNewChildElement("cRollR")->SetText(c_roll_r);
  plugin->InsertNewChildElement("cPitch0")->SetText(c_pitch_0);
  plugin->InsertNewChildElement("cPitchAlpha")->SetText(c_pitch_alpha);
  plugin->InsertNewChildElement("cPitchAbsBeta")->SetText(c_pitch_abs_beta);
  plugin->InsertNewChildElement("cPitchAlphaRate")->SetText(c_pitch_alpha_rate);
  plugin->InsertNewChildElement("cPitchQ")->SetText(c_pitch_q);
  plugin->InsertNewChildElement("cYawBeta")->SetText(c_yaw_beta);
  plugin->InsertNewChildElement("cYawP")->SetText(c_yaw_p);
  plugin->InsertNewChildElement("cYawR")->SetText(c_yaw_r);

  // Control Surfaces
  for (size_t idx = 0; idx < control_surfaces.size(); ++idx)
  {
    const auto& cs = control_surfaces.at(idx);
    const auto cs_elem = plugin->InsertNewChildElement("controlSurface");
    cs_elem->InsertNewChildElement("index")->SetText(idx);
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

void addGazeboWindPlugin(tinyxml2::XMLElement* robot, const char* ns, const char* link_name)
{
  const auto plugin = util::addGazeboPlugin(robot, "tobas_gazebo_wind_plugin", "gazebo::GazeboWindPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns);
  plugin->InsertNewChildElement("linkName")->SetText(link_name);
}

void addGazeboGroundTruthStatePlugin(tinyxml2::XMLElement* robot, const char* ns, const char* link_name)
{
  const auto plugin =
    util::addGazeboPlugin(robot, "tobas_gazebo_ground_truth_state_plugin", "gazebo::GazeboGroundTruthStatePlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns);
  plugin->InsertNewChildElement("linkName")->SetText(link_name);
}

void addRotorSpeedsPublisherPlugin(tinyxml2::XMLElement* robot, const char* ns, const QStringList& rotor_joint_names)
{
  const auto plugin = util::addGazeboPlugin(
    robot, "tobas_gazebo_rotor_speeds_publisher_plugin", "gazebo::GazeboRotorSpeedsPublisherPlugin");
  plugin->InsertNewChildElement("robotNamespace")->SetText(ns);
  util::addList(plugin, "rotorJointNames", rotor_joint_names);
}
}  // namespace setup_assistant
}  // namespace gui

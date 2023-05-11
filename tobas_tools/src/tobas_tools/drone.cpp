#include <kdl_parser/kdl_parser.hpp>

#include <dh_std_tools/math.hpp>
#include <dh_std_tools/unordered_set.hpp>
#include <dh_ros_tools/rosparam.hpp>

#include "../../include/tobas_tools/drone.hpp"

#define MIN_PIN_ID 1
#define MAX_PIN_ID 14

using namespace std;
using namespace Eigen;
using namespace KDL;

Drone::Drone()
{
}

void Drone::loadFromParam(const string& ns)
{
  getTree(ns);

  dh_ros::getParam(ns + "/battery_voltage", battery_voltage_);
  dh_ros::getParam(ns + "/active_joint_names", active_joint_names_);

  getRotorConfigs(ns);

  dh_ros::getParam(ns + "/has_fixed_wing", has_fixed_wing_);
  if (has_fixed_wing_)
  {
    getFixedWingConfig(ns);
  }
}

const Tree& Drone::tree() const
{
  return tree_;
}

const bool& Drone::hasFixedWing() const
{
  return has_fixed_wing_;
}

const double& Drone::batteryVoltage() const
{
  return battery_voltage_;
}

const vector<string>& Drone::activeJointNames() const
{
  return active_joint_names_;
}

const RotorConfigs& Drone::rotorConfigs() const
{
  return rotor_configs_;
}

const RotorConfig& Drone::rotorConfig(uint32_t rotor_idx) const
{
  return rotor_configs_[rotor_idx];
}

const FixedWingConfig& Drone::fixedWingConfig() const
{
  return fixed_wing_config_;
}

uint32_t Drone::numRotors() const
{
  return rotor_configs_.size();
}

std::vector<uint32_t> Drone::rotorConfigIdxInAxis(const Axis& axis) const
{
  vector<uint32_t> res;
  for (uint32_t i = 0; i < numRotors(); ++i)
  {
    if (rotor_configs_[i].axis == axis)
    {
      res.emplace_back(i);
    }
  }
  return res;
}

uint32_t Drone::numRotorsInAxis(const Axis& axis) const
{
  return rotorConfigIdxInAxis(axis).size();
}

double Drone::maxRotSpeed(uint32_t rotor_idx) const
{
  assert(rotor_idx < numRotors());

  return rotor_configs_[rotor_idx].kv * battery_voltage_;
}

double Drone::maxThrust(uint32_t rotor_idx) const
{
  return rotor_configs_[rotor_idx].motor_constant * dh_std::sqr(maxRotSpeed(rotor_idx));
}

void Drone::getTree(const string& ns)
{
  if (!kdl_parser::treeFromParam(ns + "/robot_description", tree_))
  {
    dh_ros::RuntimeError("Failed to get KDL tree.");
  }
}

void Drone::getRotorConfigs(const string& ns)
{
  int num_rotors;
  dh_ros::getParam(ns + "/num_rotors", num_rotors);
  rotor_configs_.resize(num_rotors);

  for (uint32_t i = 0; i < num_rotors; ++i)
  {
    getRotorConfig(ns, i);
  }
}

void Drone::getRotorConfig(const string& ns, uint32_t rotor_idx)
{
  const string prefix = ns + "/rotor_" + to_string(rotor_idx);
  auto& des = rotor_configs_[rotor_idx];

  // Link name
  dh_ros::getParam(prefix + "/link_name", des.link_name);

  // Axis
  string axis;
  dh_ros::getParam(prefix + "/axis", axis);
  if (axis == "x_positive")
  {
    des.axis = Axis::X_POSITIVE;
  }
  else if (axis == "z_positive")
  {
    des.axis = Axis::Z_POSITIVE;
  }
  else
  {
    throw dh_ros::RuntimeError("Invalid rotation axis: " + axis);
  }

  // Direction
  string direction;
  dh_ros::getParam(prefix + "/direction", direction);
  if (direction == "ccw")
  {
    des.direction = 1;
  }
  else if (direction == "cw")
  {
    des.direction = -1;
  }
  else
  {
    throw dh_ros::RuntimeError(
      "Invalid rotation direction: " + direction + ". direction must be 'cw' or 'ccw'.");
  }

  // Motor constant
  dh_ros::getParam(prefix + "/motor_constant", des.motor_constant);
  if (des.motor_constant <= 0.)
  {
    throw dh_ros::RuntimeError(
      "Invalid motor constant: " + to_string(des.motor_constant) + " N*s^2/rad^2");
  }

  // Moment constant
  dh_ros::getParam(prefix + "/moment_constant", des.moment_constant);
  if (des.moment_constant <= 0.)
  {
    throw dh_ros::RuntimeError("Invalid moment constant: " + to_string(des.moment_constant) + " m");
  }

  // KV
  dh_ros::getParam(prefix + "/kv", des.kv);
  if (des.kv <= 0.)
  {
    throw dh_ros::RuntimeError("Invalid Kv: " + to_string(des.kv) + " rpm/V");
  }

  // Index
  dh_ros::getParam(prefix + "/pin", des.pin);
  if (des.pin < MIN_PIN_ID || MAX_PIN_ID < des.pin)
  {
    throw dh_ros::RuntimeError("Invalid rotor pin number: " + to_string(des.pin));
  }

  // Pin
  dh_ros::getParam(prefix + "/pin", des.pin);
  if (des.pin < MIN_PIN_ID || MAX_PIN_ID < des.pin)
  {
    throw dh_ros::RuntimeError("Invalid rotor pin number: " + to_string(des.pin));
  }

  // ESC
  string esc_type;
  dh_ros::getParam(prefix + "/esc_type", esc_type);
  if (esc_type == "pwm")
  {
    des.esc_type = ESCType::PWM;
    dh_ros::getParam(prefix + "/pwm/frequency", des.pwm.frequency);
    dh_ros::getParam(prefix + "/pwm/min_pulse_width", des.pwm.min_pulse_width);
    dh_ros::getParam(prefix + "/pwm/max_pulse_width", des.pwm.max_pulse_width);
  }
  else if (esc_type == "dshot")
  {
    des.esc_type = ESCType::DSHOT;
    // TODO
  }
  else
  {
    throw dh_ros::RuntimeError("Unknown ESC type: " + esc_type);
  }
}

void Drone::getFixedWingConfig(const string& ns)
{
  getVehicleParameters(ns);
  getAerodynamicsCoefficients(ns);
  getControlSurfaces(ns);
}

void Drone::getVehicleParameters(const string& ns)
{
  const string prefix = ns + "/fixed_wing/vehicle";
  auto& des = fixed_wing_config_.vehicle;

  dh_ros::getParam(prefix + "/wing_surface", des.wing_surface);
  if (des.wing_surface <= 0.)
  {
    throw dh_ros::RuntimeError("wing_surface must be positive.");
  }

  dh_ros::getParam(prefix + "/wing_span", des.wing_span);
  if (des.wing_span <= 0.)
  {
    throw dh_ros::RuntimeError("wing_span must be positive.");
  }

  dh_ros::getParam(prefix + "/mean_aerodynamic_chord", des.mac);
  if (des.mac <= 0.)
  {
    throw dh_ros::RuntimeError("mean_aerodynamic_chord must be positive.");
  }

  vector<double> ac;
  dh_ros::getParam<vector<double>>(prefix + "/aerodynamic_center", ac);
  if (ac.size() != 3)
  {
    throw dh_ros::RuntimeError("Size mismatch: The size of aerodynamic_center must be 3.");
  }
  des.ac.x(ac[0]);
  des.ac.y(ac[1]);
  des.ac.z(ac[2]);

  dh_ros::getParam(prefix + "/alpha_limit/lower", des.alpha_limit.lower);
  dh_ros::getParam(prefix + "/alpha_limit/upper", des.alpha_limit.upper);
  if (!des.alpha_limit.isValid())
  {
    throw dh_ros::RuntimeError("Invalid stall angles");
  }
}

void Drone::getAerodynamicsCoefficients(const string& ns)
{
  const string prefix = ns + "/fixed_wing/aerodynamic_coefficients";
  auto& des = fixed_wing_config_.aerodynamics;

  dh_ros::getParam(prefix + "/c_lift_0", des.c_lift_0);
  if (des.c_lift_0 <= 0.)
  {
    throw dh_ros::RuntimeError("c_lift_0 must be positive.");
  }

  dh_ros::getParam(prefix + "/c_lift_alpha", des.c_lift_alpha);
  if (des.c_lift_alpha <= 0.)
  {
    throw dh_ros::RuntimeError("c_lift_alpha must be positive.");
  }

  dh_ros::getParam(prefix + "/c_drag_0", des.c_drag_0);
  if (des.c_drag_0 <= 0.)
  {
    throw dh_ros::RuntimeError("c_drag_0 must be positive.");
  }

  dh_ros::getParam(prefix + "/c_drag_alpha", des.c_drag_alpha);
  if (des.c_drag_alpha <= 0.)
  {
    throw dh_ros::RuntimeError("c_drag_alpha must be positive.");
  }

  dh_ros::getParam(prefix + "/c_side_beta", des.c_side_beta);
  if (des.c_side_beta >= 0.)
  {
    throw dh_ros::RuntimeError("c_side_beta must be negative.");
  }

  // TODO: 安定微係数の符号チェック
  dh_ros::getParam(prefix + "/c_roll_beta", des.c_roll_beta);
  dh_ros::getParam(prefix + "/c_roll_p", des.c_roll_p);
  dh_ros::getParam(prefix + "/c_roll_r", des.c_roll_r);

  dh_ros::getParam(prefix + "/c_pitch_0", des.c_pitch_0);
  dh_ros::getParam(prefix + "/c_pitch_alpha", des.c_pitch_alpha);
  dh_ros::getParam(prefix + "/c_pitch_abs_beta", des.c_pitch_abs_beta);
  dh_ros::getParam(prefix + "/c_pitch_alpha_rate", des.c_pitch_alpha_rate);
  dh_ros::getParam(prefix + "/c_pitch_q", des.c_pitch_q);

  dh_ros::getParam(prefix + "/c_yaw_beta", des.c_yaw_beta);
  dh_ros::getParam(prefix + "/c_yaw_p", des.c_yaw_p);
  dh_ros::getParam(prefix + "/c_yaw_r", des.c_yaw_r);
}

void Drone::getControlSurfaces(const string& ns)
{
  int num_cs;
  dh_ros::getParam(ns + "/num_control_surfaces", num_cs);
  fixed_wing_config_.control_surfaces.resize(num_cs);

  for (int i = 0; i < num_cs; ++i)
  {
    const string prefix = ns + "/control_surface_" + to_string(i);
    auto& des = fixed_wing_config_.control_surfaces[i];

    // indexはprefixの番号と同じ
    des.index = 0;

    dh_ros::getParam(prefix + "/angle_limit/lower", des.angle_limit.lower);
    dh_ros::getParam(prefix + "/angle_limit/upper", des.angle_limit.upper);
    if (!des.angle_limit.isValid() || !des.angle_limit.inRange(0.))
    {
      throw dh_ros::RuntimeError("Invalid range of control surface angle");
    }

    dh_ros::getParam(prefix + "/c_lift_delta", des.c_lift_delta);
    dh_ros::getParam(prefix + "/c_drag_abs_delta", des.c_drag_abs_delta);
    dh_ros::getParam(prefix + "/c_side_delta", des.c_side_delta);
    dh_ros::getParam(prefix + "/c_roll_delta", des.c_roll_delta);
    dh_ros::getParam(prefix + "/c_pitch_delta", des.c_pitch_delta);
    dh_ros::getParam(prefix + "/c_yaw_delta", des.c_yaw_delta);
  }
}

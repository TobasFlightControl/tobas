#include <kdl_parser/kdl_parser.hpp>

#include <dh_std_tools/unordered_set.hpp>
#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/rosparam.hpp>

#include "../../include/tobas_tools/drone.hpp"
#include "../../include/tobas_tools/constants.hpp"

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas
{
Drone::Drone() : is_loaded_(false)
{
}

void Drone::loadFromParam(const string& ns)
{
  getTree(ns);

  dh_ros::getParam(ns + "/active_joint_names", active_joint_names_);

  getRotorConfigs(ns);

  has_fixed_wing_ = dh_ros::match(ns + "/fixed_wing");
  if (has_fixed_wing_)
  {
    getFixedWingConfig(ns);
  }

  is_loaded_ = true;
}

const Tree& Drone::tree() const
{
  return tree_;
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

const FixedWingConfig& Drone::fixedWing() const
{
  return fixed_wing_config_;
}

const VehicleParameters& Drone::vehicle() const
{
  return fixed_wing_config_.vehicle;
}

const AerodynamicsCoefficients& Drone::aerodynamics() const
{
  return fixed_wing_config_.aerodynamics;
}

const ControlSurfaces& Drone::controlSurfaces() const
{
  return fixed_wing_config_.control_surfaces;
}

const ControlSurface& Drone::controlSurface(uint32_t cs_idx) const
{
  return fixed_wing_config_.control_surfaces[cs_idx];
}

const bool& Drone::hasFixedWing() const
{
  return has_fixed_wing_;
}

const bool& Drone::isLoaded() const
{
  return is_loaded_;
}

uint32_t Drone::numRotors() const
{
  return rotor_configs_.size();
}

uint32_t Drone::numControlSurfaces() const
{
  return fixed_wing_config_.control_surfaces.size();
}

double Drone::maxRotSpeed(uint32_t rotor_idx, double battery_voltage) const
{
  assert(battery_voltage > 0.);

  const auto max_rpm = rotor_configs_[rotor_idx].kv * battery_voltage;
  return dh_std::rpmToRadPerSec(max_rpm);
}

double Drone::maxThrust(uint32_t rotor_idx, double battery_voltage) const
{
  assert(battery_voltage > 0.);

  const auto max_rot_speed = maxRotSpeed(rotor_idx, battery_voltage);
  return rotor_configs_[rotor_idx].motor_constant * sqr(max_rot_speed);
}

double Drone::thrustToRotSpeed(uint32_t rotor_idx, double thrust) const
{
  assert(thrust >= 0.);
  return sqrt(thrust / rotor_configs_[rotor_idx].motor_constant);
}

void Drone::getTree(const string& ns)
{
  if (!kdl_parser::treeFromParam(ns + "/robot_description", tree_))
  {
    rosthrow("Failed to get KDL tree.");
  }
}

void Drone::getRotorConfigs(const string& ns)
{
  uint32_t rotor_idx = 0;
  while (dh_ros::match(ns + "/rotor_" + to_string(rotor_idx)))
  {
    rotor_configs_.push_back(getRotorConfig(ns, rotor_idx));
    ++rotor_idx;
  }
}

RotorConfig Drone::getRotorConfig(const string& ns, uint32_t rotor_idx)
{
  const string prefix = ns + "/rotor_" + to_string(rotor_idx);
  RotorConfig res;

  // Link name
  dh_ros::getParam(prefix + "/link_name", res.link_name);

  // Axis
  string axis;
  dh_ros::getParam(prefix + "/axis", axis);
  if (axis == "x_positive")
  {
    res.axis = Axis::X_POSITIVE;
  }
  else if (axis == "z_positive")
  {
    res.axis = Axis::Z_POSITIVE;
  }
  else
  {
    rosthrow("Invalid rotation axis: " << axis);
  }

  // Direction
  string direction;
  dh_ros::getParam(prefix + "/direction", direction);
  if (direction == "ccw")
  {
    res.direction = 1;
  }
  else if (direction == "cw")
  {
    res.direction = -1;
  }
  else
  {
    rosthrow("Invalid rotation direction: " << direction << ". direction must be 'cw' or 'ccw'.");
  }

  dh_ros::getParam(prefix + "/motor_constant", res.motor_constant, dh_ros::POSITIVE);
  dh_ros::getParam(prefix + "/moment_constant", res.moment_constant, dh_ros::NON_NEGATIVE);
  dh_ros::getParam(prefix + "/kv", res.kv, dh_ros::POSITIVE);

  dh_ros::getParam(prefix + "/pin", res.pin);
  if (res.pin < kMinPinId || kMaxPinId < res.pin)
  {
    rosthrow("Invalid rotor pin number: " << res.pin);
  }

  // ESC
  string esc_type;
  dh_ros::getParam(prefix + "/esc_type", esc_type);
  if (esc_type == "pwm")
  {
    res.esc_type = ESCType::PWM;
  }
  else if (esc_type == "dshot")
  {
    res.esc_type = ESCType::DSHOT;
  }
  else
  {
    rosthrow("Unknown ESC type: " << esc_type);
  }

  return res;
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

  dh_ros::getParam(prefix + "/wing_surface", des.wing_surface, dh_ros::POSITIVE);
  dh_ros::getParam(prefix + "/wing_span", des.wing_span, dh_ros::POSITIVE);
  dh_ros::getParam(prefix + "/mean_aerodynamic_chord", des.mac, dh_ros::POSITIVE);

  vector<double> ac;
  dh_ros::getParam<vector<double>>(prefix + "/aerodynamic_center", ac);
  if (ac.size() != 3)
  {
    rosthrow("Size mismatch: The size of aerodynamic_center must be 3.");
  }
  des.ac.x(ac[0]);
  des.ac.y(ac[1]);
  des.ac.z(ac[2]);

  dh_ros::getParam(prefix + "/alpha_limit/lower", des.alpha_limit.lower);
  dh_ros::getParam(prefix + "/alpha_limit/upper", des.alpha_limit.upper);
  if (!des.alpha_limit.isValid())
  {
    rosthrow("Invalid stall angles");
  }
}

void Drone::getAerodynamicsCoefficients(const string& ns)
{
  const string prefix = ns + "/fixed_wing/aerodynamic_coefficients";
  auto& des = fixed_wing_config_.aerodynamics;

  dh_ros::getParam(prefix + "/c_lift_0", des.c_lift_0, dh_ros::POSITIVE);
  dh_ros::getParam(prefix + "/c_lift_alpha", des.c_lift_alpha, dh_ros::POSITIVE);
  dh_ros::getParam(prefix + "/c_drag_0", des.c_drag_0, dh_ros::POSITIVE);
  dh_ros::getParam(prefix + "/c_drag_alpha", des.c_drag_alpha, dh_ros::POSITIVE);
  dh_ros::getParam(prefix + "/c_side_beta", des.c_side_beta, dh_ros::NEGATIVE);

  dh_ros::getParam(prefix + "/c_roll_beta", des.c_roll_beta, dh_ros::NEGATIVE);
  dh_ros::getParam(prefix + "/c_roll_p", des.c_roll_p, dh_ros::NEGATIVE);
  dh_ros::getParam(prefix + "/c_roll_r", des.c_roll_r);

  dh_ros::getParam(prefix + "/c_pitch_0", des.c_pitch_0);
  dh_ros::getParam(prefix + "/c_pitch_alpha", des.c_pitch_alpha, dh_ros::NEGATIVE);
  dh_ros::getParam(prefix + "/c_pitch_abs_beta", des.c_pitch_abs_beta);
  dh_ros::getParam(prefix + "/c_pitch_alpha_rate", des.c_pitch_alpha_rate);
  dh_ros::getParam(prefix + "/c_pitch_q", des.c_pitch_q, dh_ros::NEGATIVE);

  dh_ros::getParam(prefix + "/c_yaw_beta", des.c_yaw_beta);
  dh_ros::getParam(prefix + "/c_yaw_p", des.c_yaw_p);
  dh_ros::getParam(prefix + "/c_yaw_r", des.c_yaw_r, dh_ros::NEGATIVE);
}

void Drone::getControlSurfaces(const string& ns)
{
  uint32_t cs_idx = 0;
  while (dh_ros::match(ns + "/fixed_wing/control_surface_" + to_string(cs_idx)))
  {
    fixed_wing_config_.control_surfaces.push_back(getControlSurface(ns, cs_idx));
    ++cs_idx;
  }
}

ControlSurface Drone::getControlSurface(const string& ns, uint32_t cs_idx)
{
  const string prefix = ns + "/fixed_wing/control_surface_" + to_string(cs_idx);
  ControlSurface res;

  // indexはprefixの番号と同じ
  res.index = cs_idx;

  dh_ros::getParam(prefix + "/angle_limit/lower", res.angle_limit.lower);
  dh_ros::getParam(prefix + "/angle_limit/upper", res.angle_limit.upper);
  if (!res.angle_limit.isValid() || !res.angle_limit.inRange(0.))
  {
    rosthrow("Invalid range of control surface angle");
  }

  dh_ros::getParam(prefix + "/max_angle_rate", res.max_angle_rate, dh_ros::POSITIVE);

  dh_ros::getParam(prefix + "/c_lift_delta", res.c_lift_delta);
  dh_ros::getParam(prefix + "/c_drag_abs_delta", res.c_drag_abs_delta);
  dh_ros::getParam(prefix + "/c_side_delta", res.c_side_delta);
  dh_ros::getParam(prefix + "/c_roll_delta", res.c_roll_delta);
  dh_ros::getParam(prefix + "/c_pitch_delta", res.c_pitch_delta);
  dh_ros::getParam(prefix + "/c_yaw_delta", res.c_yaw_delta);

  return res;
}
}  // namespace tobas

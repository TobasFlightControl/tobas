#include <dh_std_tools/unordered_set.hpp>
#include <dh_std_tools/math.hpp>
#include <dh_kdl/kdl_parser.hpp>
#include <dh_ros_tools/rosparam.hpp>

#include "../include/tobas_tools/drone.hpp"
#include "../include/tobas_tools/constants.hpp"

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas
{
Drone::Drone() : is_loaded_(false)
{
}

void Drone::loadFromParam(ros::NodeHandle& nh)
{
  if (!treeFromParam("robot_description", tree_))
  {
    throw runtime_error("Failed to get KDL tree.");
  }

  dh_ros::getParam(nh, "imu_offset", imu_offset_);
  dh_ros::getParam(nh, "barometer_offset", bar_offset_);
  dh_ros::getParam(nh, "gps_offset", gps_offset_);
  dh_ros::getParam(nh, "posture_defining_joint_names", posture_defining_joints_);

  getRotorConfigs(nh);

  has_fixed_wing_ = dh_ros::match(nh, "fixed_wing");
  if (has_fixed_wing_)
  {
    getFixedWingConfig(nh);
  }

  is_loaded_ = true;
}

const Tree& Drone::tree() const
{
  return tree_;
}

const Vector3d& Drone::imuOffset() const
{
  return imu_offset_;
}

const Vector3d& Drone::barometerOffset() const
{
  return bar_offset_;
}

const Vector3d& Drone::gpsOffset() const
{
  return gps_offset_;
}

const vector<string>& Drone::postureDefiningJoints() const
{
  return posture_defining_joints_;
}

const RotorConfigs& Drone::rotorConfigs() const
{
  return rotor_configs_;
}

const RotorConfig& Drone::rotorConfig(const uint32_t& rotor_idx) const
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

const ControlSurface& Drone::controlSurface(const uint32_t& cs_idx) const
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

bool Drone::isTransformable() const
{
  return posture_defining_joints_.size() > 0;
}

uint32_t Drone::numRotors() const
{
  return rotor_configs_.size();
}

uint32_t Drone::numControlSurfaces() const
{
  return fixed_wing_config_.control_surfaces.size();
}

double Drone::thrustFromVoltage(const uint32_t& rotor_idx, const double& voltage) const
{
  assert(voltage > 0.);

  const auto rot_speed = rotSpeedFromVoltage(rotor_idx, voltage);
  return rotor_configs_[rotor_idx].motor_constant * sqr(rot_speed);
}

double Drone::voltageFromRotSpeed(const uint32_t& rotor_idx, const double& rot_speed) const
{
  assert(rot_speed >= 0.);

  const auto& a = rotor_configs_[rotor_idx].rot_speed_coefs.first;
  const auto& b = rotor_configs_[rotor_idx].rot_speed_coefs.second;
  return a * rot_speed + b * sqr(rot_speed);
}

double Drone::rotSpeedFromVoltage(const uint32_t& rotor_idx, const double& voltage) const
{
  assert(voltage >= 0.);

  const auto& a = rotor_configs_[rotor_idx].rot_speed_coefs.first;
  const auto& b = rotor_configs_[rotor_idx].rot_speed_coefs.second;
  return b > 0 ? (sqrt(sqr(a) + 4 * b * voltage) - a) / (2 * b) : voltage / a;
}

double Drone::rotSpeedFromThrust(const uint32_t& rotor_idx, const double& thrust) const
{
  assert(thrust >= 0.);
  return sqrt(thrust / rotor_configs_[rotor_idx].motor_constant);
}

void Drone::getRotorConfigs(ros::NodeHandle& nh)
{
  uint32_t num_rotors;
  dh_ros::getParam(nh, "num_rotors", num_rotors);

  for (uint32_t rotor_idx = 0; rotor_idx < num_rotors; ++rotor_idx)
  {
    rotor_configs_.push_back(getRotorConfig(nh, rotor_idx));
  }
}

RotorConfig Drone::getRotorConfig(ros::NodeHandle& nh, const uint32_t& rotor_idx)
{
  const string prefix = "rotor_" + to_string(rotor_idx);
  RotorConfig res;

  // Link name
  dh_ros::getParam(nh, prefix + "/link_name", res.link_name);

  // Axis
  string axis;
  dh_ros::getParam(nh, prefix + "/axis", axis);
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
    throw runtime_error("Invalid rotation axis: " + axis);
  }

  // Direction
  string direction;
  dh_ros::getParam(nh, prefix + "/direction", direction);
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
    throw runtime_error(
      "Invalid rotation direction: " + direction + ". direction must be 'cw' or 'ccw'.");
  }

  dh_ros::getParam(nh, prefix + "/motor_constant", res.motor_constant, dh_ros::POSITIVE);
  dh_ros::getParam(nh, prefix + "/moment_constant", res.moment_constant, dh_ros::NON_NEGATIVE);
  dh_ros::getParam(nh, prefix + "/drag_constant", res.drag_constant, dh_ros::NON_NEGATIVE);

  dh_ros::getParam(nh, prefix + "/rot_speed_coefs", res.rot_speed_coefs);
  if (res.rot_speed_coefs.first <= 0.)
  {
    throw runtime_error("The first term of 'rot_speed_coefs' must be positive.");
  }
  if (res.rot_speed_coefs.second < 0.)
  {
    throw runtime_error("The second term of 'rot_speed_coefs' must be non-negative.");
  }

  dh_ros::getParam(nh, prefix + "/pin", res.pin);
  if (res.pin < kMinPinId || kMaxPinId < res.pin)
  {
    throw runtime_error("Invalid rotor pin number: " + to_string(res.pin));
  }

  // ESC
  string esc_type;
  dh_ros::getParam(nh, prefix + "/esc_type", esc_type);
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
    throw runtime_error("Unknown ESC type: " + esc_type);
  }

  return res;
}

void Drone::getFixedWingConfig(ros::NodeHandle& nh)
{
  getVehicleParameters(nh);
  getAerodynamicsCoefficients(nh);
  getControlSurfaces(nh);
}

void Drone::getVehicleParameters(ros::NodeHandle& nh)
{
  const string prefix = "fixed_wing/vehicle";
  auto& des = fixed_wing_config_.vehicle;

  dh_ros::getParam(nh, prefix + "/wing_surface", des.wing_surface, dh_ros::POSITIVE);
  dh_ros::getParam(nh, prefix + "/wing_span", des.wing_span, dh_ros::POSITIVE);
  dh_ros::getParam(nh, prefix + "/mean_aerodynamic_chord", des.mac, dh_ros::POSITIVE);

  vector<double> ac;
  dh_ros::getParam(nh, prefix + "/aerodynamic_center", ac);
  if (ac.size() != 3)
  {
    throw runtime_error("Size mismatch: The size of aerodynamic_center must be 3.");
  }
  des.ac.x(ac[0]);
  des.ac.y(ac[1]);
  des.ac.z(ac[2]);

  dh_ros::getParam(nh, prefix + "/alpha_limit/lower", des.alpha_limit.lower);
  dh_ros::getParam(nh, prefix + "/alpha_limit/upper", des.alpha_limit.upper);
  if (!des.alpha_limit.isValid())
  {
    throw runtime_error("Invalid stall angles");
  }
}

void Drone::getAerodynamicsCoefficients(ros::NodeHandle& nh)
{
  const string prefix = "fixed_wing/aerodynamic_coefficients";
  auto& des = fixed_wing_config_.aerodynamics;

  dh_ros::getParam(nh, prefix + "/c_lift_0", des.c_lift_0, dh_ros::POSITIVE);
  dh_ros::getParam(nh, prefix + "/c_lift_alpha", des.c_lift_alpha, dh_ros::POSITIVE);
  dh_ros::getParam(nh, prefix + "/c_drag_0", des.c_drag_0, dh_ros::POSITIVE);
  dh_ros::getParam(nh, prefix + "/c_drag_alpha", des.c_drag_alpha, dh_ros::POSITIVE);
  dh_ros::getParam(nh, prefix + "/c_side_beta", des.c_side_beta, dh_ros::NEGATIVE);

  dh_ros::getParam(nh, prefix + "/c_roll_beta", des.c_roll_beta, dh_ros::NEGATIVE);
  dh_ros::getParam(nh, prefix + "/c_roll_p", des.c_roll_p, dh_ros::NEGATIVE);
  dh_ros::getParam(nh, prefix + "/c_roll_r", des.c_roll_r);

  dh_ros::getParam(nh, prefix + "/c_pitch_0", des.c_pitch_0);
  dh_ros::getParam(nh, prefix + "/c_pitch_alpha", des.c_pitch_alpha, dh_ros::NEGATIVE);
  dh_ros::getParam(nh, prefix + "/c_pitch_abs_beta", des.c_pitch_abs_beta);
  dh_ros::getParam(nh, prefix + "/c_pitch_alpha_rate", des.c_pitch_alpha_rate);
  dh_ros::getParam(nh, prefix + "/c_pitch_q", des.c_pitch_q, dh_ros::NEGATIVE);

  dh_ros::getParam(nh, prefix + "/c_yaw_beta", des.c_yaw_beta);
  dh_ros::getParam(nh, prefix + "/c_yaw_p", des.c_yaw_p);
  dh_ros::getParam(nh, prefix + "/c_yaw_r", des.c_yaw_r, dh_ros::NEGATIVE);
}

void Drone::getControlSurfaces(ros::NodeHandle& nh)
{
  // fixed_wing/controll_surface_0などにはnh.searchParam()が使えないため，
  // 制御面の個数を明示的にパラメータサーバから取得する．
  uint32_t num_cs;
  dh_ros::getParam(nh, "fixed_wing/num_control_surfaces", num_cs);

  for (uint32_t cs_idx = 0; cs_idx < num_cs; ++cs_idx)
  {
    fixed_wing_config_.control_surfaces.push_back(getControlSurface(nh, cs_idx));
  }
}

ControlSurface Drone::getControlSurface(ros::NodeHandle& nh, const uint32_t& cs_idx)
{
  const string prefix = "fixed_wing/control_surface_" + to_string(cs_idx);
  ControlSurface res;

  // indexはprefixの番号と同じ
  res.index = cs_idx;

  dh_ros::getParam(nh, prefix + "/angle_limit/lower", res.angle_limit.lower);
  dh_ros::getParam(nh, prefix + "/angle_limit/upper", res.angle_limit.upper);
  if (!res.angle_limit.isValid() || !res.angle_limit.inRange(0.))
  {
    throw runtime_error("Invalid range of control surface angle");
  }

  dh_ros::getParam(nh, prefix + "/max_angle_rate", res.max_angle_rate, dh_ros::POSITIVE);

  dh_ros::getParam(nh, prefix + "/c_lift_delta", res.c_lift_delta);
  dh_ros::getParam(nh, prefix + "/c_drag_abs_delta", res.c_drag_abs_delta);
  dh_ros::getParam(nh, prefix + "/c_side_delta", res.c_side_delta);
  dh_ros::getParam(nh, prefix + "/c_roll_delta", res.c_roll_delta);
  dh_ros::getParam(nh, prefix + "/c_pitch_delta", res.c_pitch_delta);
  dh_ros::getParam(nh, prefix + "/c_yaw_delta", res.c_yaw_delta);

  return res;
}
}  // namespace tobas

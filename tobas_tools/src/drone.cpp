#include <tobas_std_tools/unordered_set.hpp>
#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/string.hpp>
#include <tobas_std_tools/console.hpp>
#include <tobas_kdl/kdl_parser.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_ros_tools/exception.hpp>

#include "../include/tobas_tools/drone.hpp"
#include "../include/tobas_tools/constants.hpp"

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas
{
Drone::Drone()
{
  TOBAS_DEBUG("Drone::Drone");
}

void Drone::loadFromParam(ros::NodeHandle& nh)
{
  TOBAS_DEBUG("Drone::loadFromParam");

  if (!treeFromParam(kRobotDescriptionParam, tree_))
    ROS_THROW("Failed to get KDL tree.");

  getJointConfigs(nh);
  getRotorConfigs(nh);

  has_fixed_wing_ = tobas_ros::match(nh, "fixed_wing");
  if (has_fixed_wing_)
    getFixedWingConfig(nh);

  is_loaded_ = true;
}

vector<string> Drone::postureDefiningJointNames() const
{
  vector<string> res;
  for (const auto& joint : joints_)
    res.push_back(joint.name);
  return res;
}

double Drone::maxRotSpeed(const size_t& rotor_idx, const double& battery_voltage) const
{
  return min(rotors_.at(rotor_idx).max_rot_speed, rotSpeedFromVoltage(rotor_idx, battery_voltage));
}

double Drone::minRotSpeed(const size_t& rotor_idx, const double& battery_voltage) const
{
  const auto min_voltage = battery_voltage * kArmThrottle;
  return min(rotors_.at(rotor_idx).max_rot_speed, rotSpeedFromVoltage(rotor_idx, min_voltage));
}

double Drone::maxMechanicalThrust(const size_t& rotor_idx) const
{
  const auto& rotor = rotors_.at(rotor_idx);
  return rotor.motor_constant * sqr(rotor.max_rot_speed);
}

double Drone::maxThrust(const size_t& rotor_idx, const double& battery_voltage) const
{
  // 機械的な限界とエネルギー的な限界の最小値を計算
  return min(maxMechanicalThrust(rotor_idx), thrustFromVoltage(rotor_idx, battery_voltage));
}

double Drone::minThrust(const size_t& rotor_idx, const double& battery_voltage) const
{
  const auto min_voltage = battery_voltage * kArmThrottle;
  return min(maxMechanicalThrust(rotor_idx), thrustFromVoltage(rotor_idx, min_voltage));
}

double Drone::thrustFromRotSpeed(const size_t& rotor_idx, const double& tar_speed) const
{
  return rotors_[rotor_idx].motor_constant * sqr(tar_speed);
}

double Drone::thrustFromVoltage(const size_t& rotor_idx, const double& voltage) const
{
  assert(voltage > 0);

  const auto tar_speed = rotSpeedFromVoltage(rotor_idx, voltage);
  return thrustFromRotSpeed(rotor_idx, tar_speed);
}

double Drone::voltageFromRotSpeed(const size_t& rotor_idx, const double& tar_speed) const
{
  assert(tar_speed >= 0);

  const auto& a = rotors_[rotor_idx].rot_speed_coefs.first;
  const auto& b = rotors_[rotor_idx].rot_speed_coefs.second;
  return a * tar_speed + b * sqr(tar_speed);
}

double Drone::rotSpeedFromVoltage(const size_t& rotor_idx, const double& voltage) const
{
  assert(voltage >= 0);

  const auto& a = rotors_[rotor_idx].rot_speed_coefs.first;
  const auto& b = rotors_[rotor_idx].rot_speed_coefs.second;
  return b > 0 ? (sqrt(sqr(a) + 4 * b * voltage) - a) / (2 * b) : voltage / a;
}

double Drone::rotSpeedFromThrust(const size_t& rotor_idx, const double& thrust) const
{
  assert(thrust >= 0);
  return sqrt(thrust / rotors_[rotor_idx].motor_constant);
}

double Drone::throttleFromRotSpeed(
  const size_t& rotor_idx,
  const double& tar_speed,
  const double& battery_voltage) const
{
  assert(tar_speed >= 0);

  const auto voltage = voltageFromRotSpeed(rotor_idx, tar_speed);
  return voltage / battery_voltage;
}

double Drone::throttleFromThrust(
  const size_t& rotor_idx,
  const double& thrust,
  const double& battery_voltage) const
{
  assert(thrust >= 0);

  const auto tar_speed = rotSpeedFromThrust(rotor_idx, thrust);
  return throttleFromRotSpeed(rotor_idx, tar_speed, battery_voltage);
}

void Drone::getJointConfigs(ros::NodeHandle& nh)
{
  TOBAS_DEBUG("Drone::getJointConfigs");

  size_t num_joints;
  tobas_ros::getParam(nh, "num_joints", num_joints);

  for (size_t joint_idx = 0; joint_idx < num_joints; ++joint_idx)
    joints_.push_back(getJointConfig(nh, joint_idx));
}

JointConfig Drone::getJointConfig(ros::NodeHandle& nh, const size_t& joint_idx)
{
  TOBAS_DEBUG("Drone::getJointConfigs(" << joint_idx << ")");

  const string prefix = "joint_" + to_string(joint_idx);
  JointConfig res;

  tobas_ros::getParam(nh, prefix + "/name", res.name);

  tobas_ros::getParam(nh, prefix + "/home_position", res.home_pos);
  tobas_ros::getParam(nh, prefix + "/min_position", res.min_pos);
  tobas_ros::getParam(nh, prefix + "/max_position", res.max_pos);
  if (!(res.min_pos <= res.home_pos && res.home_pos <= res.max_pos))
    ROS_THROW("Invalid value for joint '" << res.name << "'.");

  string cmd_type;
  tobas_ros::getParam(nh, prefix + "/command_type", cmd_type);
  if (cmd_type == "position")
    res.cmd_type = JointConfig::POSITION;
  else if (cmd_type == "velocity")
    res.cmd_type = JointConfig::VELOCITY;
  else if (cmd_type == "effort")
    res.cmd_type = JointConfig::EFFORT;
  else
    ROS_THROW("Invalid command type: " << cmd_type);

  return res;
}

void Drone::getRotorConfigs(ros::NodeHandle& nh)
{
  TOBAS_DEBUG("Drone::getRotorConfigs");

  size_t num_rotors;
  tobas_ros::getParam(nh, "num_rotors", num_rotors);

  for (size_t rotor_idx = 0; rotor_idx < num_rotors; ++rotor_idx)
    rotors_.push_back(getRotorConfig(nh, rotor_idx));
}

RotorConfig Drone::getRotorConfig(ros::NodeHandle& nh, const size_t& rotor_idx)
{
  TOBAS_DEBUG("Drone::getRotorConfigs(" << rotor_idx << ")");

  const string prefix = "rotor_" + to_string(rotor_idx);
  RotorConfig res;

  // Link name
  tobas_ros::getParam(nh, prefix + "/link_name", res.link_name);

  // Direction
  string direction;
  tobas_ros::getParam(nh, prefix + "/direction", direction);
  direction = tobas_std::toLower(direction);
  if (direction == "ccw")
    res.direction = 1;
  else if (direction == "cw")
    res.direction = -1;
  else
    ROS_THROW("Invalid rotation direction: " << direction << ". direction must be 'cw' or 'ccw'.");

  // Axis
  string axis;
  tobas_ros::getParam(nh, prefix + "/axis", axis);
  axis = tobas_std::toLower(axis);
  if (axis == "x_positive")
    res.axis = Axis::X_POSITIVE;
  else if (axis == "z_positive")
    res.axis = Axis::Z_POSITIVE;
  else
    res.axis = Axis::UNKNOWN;

  // ESC signal mode
  string esc_signal_mode;
  tobas_ros::getParam(nh, prefix + "/esc_signal_mode", esc_signal_mode);
  esc_signal_mode = tobas_std::toLower(esc_signal_mode);
  if (esc_signal_mode == "blheli_open_loop")
    res.esc_signal_mode = EscSignalMode::BLHELI_OPEN_LOOP;
  else if (esc_signal_mode == "blheli_closed_loop_low_range")
    res.esc_signal_mode = EscSignalMode::BLHELI_CLOSED_LOOP_LOW_RANGE;
  else if (esc_signal_mode == "blheli_closed_loop_mid_range")
    res.esc_signal_mode = EscSignalMode::BLHELI_CLOSED_LOOP_MID_RANGE;
  else if (esc_signal_mode == "blheli_closed_loop_high_range")
    res.esc_signal_mode = EscSignalMode::BLHELI_CLOSED_LOOP_HIGH_RANGE;
  else
    ROS_THROW("Invalid ESC signal mode: " << esc_signal_mode);

  // The number of poles
  tobas_ros::getParam(nh, prefix + "/num_poles", res.num_poles);
  if (res.num_poles == 0)
    ROS_THROW("The number of poles cannot be 0.");
  if (res.num_poles % 2 == 1)
    ROS_THROW("The number of poles must be even.");

  tobas_ros::getParam(nh, prefix + "/max_rot_speed", res.max_rot_speed, tobas_ros::NON_NEGATIVE);
  tobas_ros::getParam(nh, prefix + "/motor_constant", res.motor_constant, tobas_ros::POSITIVE);
  tobas_ros::getParam(
    nh, prefix + "/moment_constant", res.moment_constant, tobas_ros::NON_NEGATIVE);
  tobas_ros::getParam(nh, prefix + "/drag_constant", res.drag_constant, tobas_ros::NON_NEGATIVE);

  tobas_ros::getParam(nh, prefix + "/rot_speed_coefs", res.rot_speed_coefs);
  if (res.rot_speed_coefs.first <= 0)
    ROS_THROW("The first term of 'rot_speed_coefs' must be positive.");
  if (res.rot_speed_coefs.second < 0)
    ROS_THROW("The second term of 'rot_speed_coefs' must be non-negative.");

  tobas_ros::getParam(nh, prefix + "/pin", res.pin);
  if (res.pin < kMinPinId || kMaxPinId < res.pin)
    ROS_THROW("Invalid rotor pin number: " << res.pin);

  return res;
}

void Drone::getFixedWingConfig(ros::NodeHandle& nh)
{
  TOBAS_DEBUG("Drone::getFixedWingConfig");

  getVehicleParameters(nh);
  getAerodynamicsCoefficients(nh);
  getControlSurfaces(nh);
}

void Drone::getVehicleParameters(ros::NodeHandle& nh)
{
  TOBAS_DEBUG("Drone::getVehicleParameters");

  const string prefix = "fixed_wing/vehicle";
  auto& des = fixed_wing_.vehicle;

  tobas_ros::getParam(nh, prefix + "/wing_surface", des.wing_surface, tobas_ros::POSITIVE);
  tobas_ros::getParam(nh, prefix + "/wing_span", des.wing_span, tobas_ros::POSITIVE);
  tobas_ros::getParam(nh, prefix + "/mean_aerodynamic_chord", des.mac, tobas_ros::POSITIVE);

  vector<double> ac;
  tobas_ros::getParam(nh, prefix + "/aerodynamic_center", ac);
  if (ac.size() != 3)
    ROS_THROW("Size mismatch: The size of aerodynamic_center must be 3.");
  des.ac.x(ac[0]);
  des.ac.y(ac[1]);
  des.ac.z(ac[2]);

  tobas_ros::getParam(nh, prefix + "/alpha_limit/lower", des.alpha_limit.lower);
  tobas_ros::getParam(nh, prefix + "/alpha_limit/upper", des.alpha_limit.upper);
  if (!des.alpha_limit.isValid())
    ROS_THROW("Invalid stall angles");
}

void Drone::getAerodynamicsCoefficients(ros::NodeHandle& nh)
{
  TOBAS_DEBUG("Drone::getAerodynamicsCoefficients");

  const string prefix = "fixed_wing/aerodynamic_coefficients";
  auto& des = fixed_wing_.aerodynamics;

  tobas_ros::getParam(nh, prefix + "/c_lift_0", des.c_lift_0, tobas_ros::POSITIVE);
  tobas_ros::getParam(nh, prefix + "/c_lift_alpha", des.c_lift_alpha, tobas_ros::POSITIVE);
  tobas_ros::getParam(nh, prefix + "/c_drag_0", des.c_drag_0, tobas_ros::POSITIVE);
  tobas_ros::getParam(nh, prefix + "/c_drag_alpha", des.c_drag_alpha, tobas_ros::POSITIVE);
  tobas_ros::getParam(nh, prefix + "/c_side_beta", des.c_side_beta, tobas_ros::NEGATIVE);

  tobas_ros::getParam(nh, prefix + "/c_roll_beta", des.c_roll_beta, tobas_ros::NEGATIVE);
  tobas_ros::getParam(nh, prefix + "/c_roll_p", des.c_roll_p, tobas_ros::NEGATIVE);
  tobas_ros::getParam(nh, prefix + "/c_roll_r", des.c_roll_r);

  tobas_ros::getParam(nh, prefix + "/c_pitch_0", des.c_pitch_0);
  tobas_ros::getParam(nh, prefix + "/c_pitch_alpha", des.c_pitch_alpha, tobas_ros::NEGATIVE);
  tobas_ros::getParam(nh, prefix + "/c_pitch_abs_beta", des.c_pitch_abs_beta);
  tobas_ros::getParam(nh, prefix + "/c_pitch_alpha_rate", des.c_pitch_alpha_rate);
  tobas_ros::getParam(nh, prefix + "/c_pitch_q", des.c_pitch_q, tobas_ros::NEGATIVE);

  tobas_ros::getParam(nh, prefix + "/c_yaw_beta", des.c_yaw_beta);
  tobas_ros::getParam(nh, prefix + "/c_yaw_p", des.c_yaw_p);
  tobas_ros::getParam(nh, prefix + "/c_yaw_r", des.c_yaw_r, tobas_ros::NEGATIVE);
}

void Drone::getControlSurfaces(ros::NodeHandle& nh)
{
  TOBAS_DEBUG("Drone::getControlSurfaces");

  // fixed_wing/controll_surface_0などにはnh.searchParam()が使えないため，
  // 制御面の個数を明示的にパラメータサーバから取得する．
  size_t num_cs;
  tobas_ros::getParam(nh, "fixed_wing/num_control_surfaces", num_cs);

  for (size_t cs_idx = 0; cs_idx < num_cs; ++cs_idx)
    fixed_wing_.control_surfaces.push_back(getControlSurface(nh, cs_idx));
}

ControlSurface Drone::getControlSurface(ros::NodeHandle& nh, const size_t& cs_idx)
{
  TOBAS_DEBUG("Drone::getRotorConfigs(" << cs_idx << ")");

  const string prefix = "fixed_wing/control_surface_" + to_string(cs_idx);
  ControlSurface res;

  // indexはprefixの番号と同じ
  res.index = cs_idx;

  tobas_ros::getParam(nh, prefix + "/angle_limit/lower", res.angle_limit.lower);
  tobas_ros::getParam(nh, prefix + "/angle_limit/upper", res.angle_limit.upper);
  if (!res.angle_limit.isValid() || !res.angle_limit.inRange(0))
    ROS_THROW("Invalid range of control surface angle");

  tobas_ros::getParam(nh, prefix + "/max_angle_rate", res.max_angle_rate, tobas_ros::POSITIVE);

  tobas_ros::getParam(nh, prefix + "/c_lift_delta", res.c_lift_delta);
  tobas_ros::getParam(nh, prefix + "/c_drag_abs_delta", res.c_drag_abs_delta);
  tobas_ros::getParam(nh, prefix + "/c_side_delta", res.c_side_delta);
  tobas_ros::getParam(nh, prefix + "/c_roll_delta", res.c_roll_delta);
  tobas_ros::getParam(nh, prefix + "/c_pitch_delta", res.c_pitch_delta);
  tobas_ros::getParam(nh, prefix + "/c_yaw_delta", res.c_yaw_delta);

  return res;
}
}  // namespace tobas

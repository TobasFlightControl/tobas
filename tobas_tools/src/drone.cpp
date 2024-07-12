#include <tobas_std_tools/unordered_set.hpp>
#include <tobas_math/core.hpp>
#include <tobas_std_tools/console.hpp>
#include <tobas_kdl/kdl_parser.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_ros_tools/exception.hpp>

#include "../include/tobas_tools/drone.hpp"

using namespace std;
using namespace Eigen;

namespace tobas
{
Drone::Drone()
{
  PRINT_DEBUG("Drone::Drone");
}

void Drone::loadFromParam(ros::NodeHandle& nh)
{
  PRINT_DEBUG("Drone::loadFromParam");

  ROS_CHECK(nh, treeFromParam(kRobotDescriptionParam, tree_), "Failed to get tobas_kdl tree.")

  tobas_ros::getParam(nh, "drone_name", drone_name_);

  getBatteryConfig(nh);
  getJointConfigs(nh);
  getRotorConfigs(nh);

  has_fixed_wing_ = tobas_ros::match(nh, "fixed_wing");
  if (has_fixed_wing_)
    getFixedWingConfig(nh);

  is_loaded_ = true;
}

void Drone::getBatteryConfig(ros::NodeHandle& nh)
{
  PRINT_DEBUG("Drone::getBatteryConfig");

  const string prefix = "battery";

  tobas_ros::getParam(nh, prefix + "/nominal_voltage", battery_.nominal_voltage);
  tobas_ros::getParam(nh, prefix + "/max_voltage", battery_.max_voltage);
  tobas_ros::getParam(nh, prefix + "/sag_voltage", battery_.sag_voltage);
  ROS_CHECK(
    nh,
    0 < battery_.sag_voltage && battery_.sag_voltage < battery_.nominal_voltage
      && battery_.nominal_voltage < battery_.max_voltage,
    "Invalid battery configuration.");

  tobas_ros::getParam(nh, prefix + "/max_current", battery_.max_current, tobas_ros::POSITIVE);
}

void Drone::getJointConfigs(ros::NodeHandle& nh)
{
  PRINT_DEBUG("Drone::getJointConfigs");

  size_t num_joints;
  tobas_ros::getParam(nh, "num_joints", num_joints);

  for (size_t jnt_idx = 0; jnt_idx < num_joints; ++jnt_idx)
    getJointConfig(nh, jnt_idx);
}

void Drone::getJointConfig(ros::NodeHandle& nh, size_t jnt_idx)
{
  PRINT_DEBUG("Drone::getJointConfigs(" << jnt_idx << ")");

  const string prefix = "joint_" + to_string(jnt_idx);
  string name;
  JointConfig cfg;

  tobas_ros::getParam(nh, prefix + "/name", name);
  tobas_ros::getParam(nh, prefix + "/home_position", cfg.home_pos);
  tobas_ros::getParam(nh, prefix + "/min_position", cfg.min_pos);
  tobas_ros::getParam(nh, prefix + "/max_position", cfg.max_pos);
  ROS_CHECK(
    nh, cfg.min_pos <= cfg.home_pos && cfg.home_pos <= cfg.max_pos, "Invalid value for joint '" << name << "'.");

  string cmd_type;
  tobas_ros::getParam(nh, prefix + "/command_type", cmd_type);
  if (cmd_type == "position")
    cfg.cmd_type = JointConfig::POSITION;
  else if (cmd_type == "velocity")
    cfg.cmd_type = JointConfig::VELOCITY;
  else if (cmd_type == "effort")
    cfg.cmd_type = JointConfig::EFFORT;
  else
    ROS_EXIT(nh, "Invalid command type: " << cmd_type);

  joint_map_[name] = cfg;
}

void Drone::getRotorConfigs(ros::NodeHandle& nh)
{
  PRINT_DEBUG("Drone::getRotorConfigs");

  size_t num_rotors;
  tobas_ros::getParam(nh, "num_rotors", num_rotors);

  for (size_t rotor_idx = 0; rotor_idx < num_rotors; ++rotor_idx)
    rotors_.push_back(getRotorConfig(nh, rotor_idx));
}

RotorConfig Drone::getRotorConfig(ros::NodeHandle& nh, size_t rotor_idx)
{
  PRINT_DEBUG("Drone::getRotorConfigs(" << rotor_idx << ")");

  const string prefix = "rotor_" + to_string(rotor_idx);
  RotorConfig res;

  // Link name
  tobas_ros::getParam(nh, prefix + "/link_name", res.link_name);

  // Turning Direction
  string direction;
  tobas_ros::getParam(nh, prefix + "/direction", direction);
  if (direction == CW.name)
    res.direction = CW;
  else if (direction == CCW.name)
    res.direction = CCW;
  else
    ROS_EXIT(nh, "Invalid rotation direction: " << direction << ". It must be 'CW' or 'CCW'.");

  // Rotor Axis
  string axis;
  tobas_ros::getParam(nh, prefix + "/axis", axis);
  if (axis == X_POSITIVE.name)
    res.axis = X_POSITIVE;
  else if (axis == Z_POSITIVE.name)
    res.axis = Z_POSITIVE;
  else
    res.axis = UNKNOWN;

  // ESC signal mode
  string esc_mode;
  tobas_ros::getParam(nh, prefix + "/esc_mode", esc_mode);
  if (esc_mode == BLHELI_OPEN_LOOP.name)
    res.esc_mode = BLHELI_OPEN_LOOP;
  else if (esc_mode == BLHELI_CLOSED_LOOP_LOW_RANGE.name)
    res.esc_mode = BLHELI_CLOSED_LOOP_LOW_RANGE;
  else if (esc_mode == BLHELI_CLOSED_LOOP_MID_RANGE.name)
    res.esc_mode = BLHELI_CLOSED_LOOP_MID_RANGE;
  else if (esc_mode == BLHELI_CLOSED_LOOP_HIGH_RANGE.name)
    res.esc_mode = BLHELI_CLOSED_LOOP_HIGH_RANGE;
  else
    ROS_EXIT(nh, "Invalid ESC signal mode: " << esc_mode);

  // The number of poles
  tobas_ros::getParam(nh, prefix + "/num_poles", res.num_poles);
  ROS_CHECK(nh, res.num_poles > 0, "The number of poles must be positive.");
  ROS_CHECK(nh, res.num_poles % 2 == 0, "The number of poles must be even.");

  tobas_ros::getParam(nh, prefix + "/max_rot_speed", res.max_rot_speed, tobas_ros::NON_NEGATIVE);
  tobas_ros::getParam(nh, prefix + "/motor_constant", res.motor_constant, tobas_ros::POSITIVE);
  tobas_ros::getParam(nh, prefix + "/moment_constant", res.moment_constant, tobas_ros::NON_NEGATIVE);
  tobas_ros::getParam(nh, prefix + "/drag_constant", res.drag_constant, tobas_ros::NON_NEGATIVE);

  tobas_ros::getParam(nh, prefix + "/rot_speed_coefs", res.rot_speed_coefs);
  ROS_CHECK(nh, res.rot_speed_coefs.first > 0, "The first term of 'rot_speed_coefs' must be positive.");
  ROS_CHECK(nh, res.rot_speed_coefs.second >= 0, "The second term of 'rot_speed_coefs' must be non-negative.");

  tobas_ros::getParam(nh, prefix + "/channel", res.channel);

  return res;
}

void Drone::getFixedWingConfig(ros::NodeHandle& nh)
{
  PRINT_DEBUG("Drone::getFixedWingConfig");

  getVehicleParameters(nh);
  getAerodynamicsCoefficients(nh);
  getControlSurfaces(nh);
}

void Drone::getVehicleParameters(ros::NodeHandle& nh)
{
  PRINT_DEBUG("Drone::getVehicleParameters");

  const string prefix = "fixed_wing/vehicle";
  auto& des = fixed_wing_.vehicle;

  tobas_ros::getParam(nh, prefix + "/wing_surface", des.wing_surface, tobas_ros::POSITIVE);
  tobas_ros::getParam(nh, prefix + "/wing_span", des.wing_span, tobas_ros::POSITIVE);
  tobas_ros::getParam(nh, prefix + "/mean_aerodynamic_chord", des.mac, tobas_ros::POSITIVE);
  tobas_ros::getParam(nh, prefix + "/aerodynamic_center", des.ac.data);
  tobas_ros::getParam(nh, prefix + "/alpha_limit/lower", des.alpha_limit.lower);
  tobas_ros::getParam(nh, prefix + "/alpha_limit/upper", des.alpha_limit.upper);

  ROS_CHECK(nh, des.alpha_limit.isValid(), "Invalid stall angles");
}

void Drone::getAerodynamicsCoefficients(ros::NodeHandle& nh)
{
  PRINT_DEBUG("Drone::getAerodynamicsCoefficients");

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
  PRINT_DEBUG("Drone::getControlSurfaces");

  // fixed_wing/controll_surface_0などにはnh.searchParam()が使えないため，
  // 制御面の個数を明示的にパラメータサーバから取得する．
  size_t num_cs;
  tobas_ros::getParam(nh, "fixed_wing/num_control_surfaces", num_cs);

  for (size_t cs_idx = 0; cs_idx < num_cs; ++cs_idx)
    fixed_wing_.control_surfaces.push_back(getControlSurface(nh, cs_idx));
}

ControlSurface Drone::getControlSurface(ros::NodeHandle& nh, size_t cs_idx)
{
  PRINT_DEBUG("Drone::getRotorConfigs(" << cs_idx << ")");

  const string prefix = "fixed_wing/control_surface_" + to_string(cs_idx);
  ControlSurface res;

  // indexはprefixの番号と同じ
  res.index = cs_idx;

  tobas_ros::getParam(nh, prefix + "/angle_limit/lower", res.angle_limit.lower);
  tobas_ros::getParam(nh, prefix + "/angle_limit/upper", res.angle_limit.upper);
  ROS_CHECK(nh, res.angle_limit.isValid() && res.angle_limit.inRange(0), "Invalid range of control surface angle");

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

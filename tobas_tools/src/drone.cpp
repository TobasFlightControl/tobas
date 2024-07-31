#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/console.hpp>
#include <tobas_kdl/kdl_parser.hpp>
#include <tobas_ros2_tools/rosparam.hpp>

#include "../include/tobas_tools/drone.hpp"

using namespace std;
using namespace Eigen;

namespace tobas
{
Drone::Drone()
{
  PRINT_DEBUG("Drone::Drone");
}

void Drone::loadFromParam(rclcpp::Node::SharedPtr node)
{
  PRINT_DEBUG("Drone::loadFromParam");

  TOBAS_CHECK(treeFromParam(kRobotDescriptionParam, tree_))

  ros2::getParam(node, "drone_name", drone_name_);

  getBatteryConfig(node);
  getJointConfigs(node);
  getRotorConfigs(node);

  ros2::getParam(node, "fixed_wing", has_fixed_wing_);
  if (has_fixed_wing_)
    getFixedWingConfig(node);

  is_loaded_ = true;
}

void Drone::getBatteryConfig(rclcpp::Node::SharedPtr node)
{
  PRINT_DEBUG("Drone::getBatteryConfig");

  const string prefix = "battery";

  ros2::getParam(node, prefix + "/nominal_voltage", battery_.nominal_voltage);
  ros2::getParam(node, prefix + "/max_voltage", battery_.max_voltage);
  ros2::getParam(node, prefix + "/sag_voltage", battery_.sag_voltage);
  TOBAS_CHECK(
    0 < battery_.sag_voltage && battery_.sag_voltage < battery_.nominal_voltage
    && battery_.nominal_voltage < battery_.max_voltage);

  ros2::getParam(node, prefix + "/max_current", battery_.max_current, ros2::POSITIVE);
}

void Drone::getJointConfigs(rclcpp::Node::SharedPtr node)
{
  PRINT_DEBUG("Drone::getJointConfigs");

  size_t num_joints;
  ros2::getParam(node, "num_joints", num_joints);

  joint_map_.clear();

  for (size_t jnt_idx = 0; jnt_idx < num_joints; ++jnt_idx)
    getJointConfig(node, jnt_idx);
}

void Drone::getJointConfig(rclcpp::Node::SharedPtr node, size_t jnt_idx)
{
  PRINT_DEBUG("Drone::getJointConfig(" << jnt_idx << ")");

  const string prefix = "joint_" + to_string(jnt_idx);
  string name;
  JointConfig cfg;

  ros2::getParam(node, prefix + "/name", name);
  ros2::getParam(node, prefix + "/home_position", cfg.home_pos);
  ros2::getParam(node, prefix + "/min_position", cfg.min_pos);
  ros2::getParam(node, prefix + "/max_position", cfg.max_pos);
  TOBAS_CHECK(cfg.min_pos <= cfg.home_pos && cfg.home_pos <= cfg.max_pos);

  string cmd_type;
  ros2::getParam(node, prefix + "/command_type", cmd_type);
  if (cmd_type == "position")
    cfg.cmd_type = JointConfig::POSITION;
  else if (cmd_type == "velocity")
    cfg.cmd_type = JointConfig::VELOCITY;
  else if (cmd_type == "effort")
    cfg.cmd_type = JointConfig::EFFORT;
  else
    throw runtime_error("Invalid command type: " + cmd_type);

  joint_map_[name] = cfg;
}

void Drone::getRotorConfigs(rclcpp::Node::SharedPtr node)
{
  PRINT_DEBUG("Drone::getRotorConfigs");

  size_t num_rotors;
  ros2::getParam(node, "num_rotors", num_rotors);

  rotors_.resize(num_rotors);

  for (size_t rotor_idx = 0; rotor_idx < num_rotors; ++rotor_idx)
    getRotorConfig(node, rotor_idx, rotors_.at(rotor_idx));
}

void Drone::getRotorConfig(rclcpp::Node::SharedPtr node, size_t rotor_idx, RotorConfig& des)
{
  PRINT_DEBUG("Drone::getRotorConfig(" << rotor_idx << ")");

  const string prefix = "rotor_" + to_string(rotor_idx);

  // Link name
  ros2::getParam(node, prefix + "/link_name", des.link_name);

  // Turning Direction
  string direction;
  ros2::getParam(node, prefix + "/direction", direction);
  if (direction == CW.name)
    des.direction = CW;
  else if (direction == CCW.name)
    des.direction = CCW;
  else
    throw runtime_error("Invalid rotation direction: " + direction + ". It must be 'CW' or 'CCW'.");

  // Rotor Axis
  string axis;
  ros2::getParam(node, prefix + "/axis", axis);
  if (axis == X_POSITIVE.name)
    des.axis = X_POSITIVE;
  else if (axis == Z_POSITIVE.name)
    des.axis = Z_POSITIVE;
  else
    des.axis = UNKNOWN;

  // ESC signal mode
  string esc_mode;
  ros2::getParam(node, prefix + "/esc_mode", esc_mode);
  if (esc_mode == BLHELI_OPEN_LOOP.name)
    des.esc_mode = BLHELI_OPEN_LOOP;
  else if (esc_mode == BLHELI_CLOSED_LOOP_LOW_RANGE.name)
    des.esc_mode = BLHELI_CLOSED_LOOP_LOW_RANGE;
  else if (esc_mode == BLHELI_CLOSED_LOOP_MID_RANGE.name)
    des.esc_mode = BLHELI_CLOSED_LOOP_MID_RANGE;
  else if (esc_mode == BLHELI_CLOSED_LOOP_HIGH_RANGE.name)
    des.esc_mode = BLHELI_CLOSED_LOOP_HIGH_RANGE;
  else
    throw runtime_error("Invalid ESC signal mode: " + esc_mode);

  // The number of poles
  ros2::getParam(node, prefix + "/num_poles", des.num_poles);
  TOBAS_CHECK(des.num_poles > 0);
  TOBAS_CHECK(des.num_poles % 2 == 0);

  ros2::getParam(node, prefix + "/max_rot_speed", des.max_rot_speed, ros2::NON_NEGATIVE);
  ros2::getParam(node, prefix + "/motor_constant", des.motor_constant, ros2::POSITIVE);
  ros2::getParam(node, prefix + "/moment_constant", des.moment_constant, ros2::NON_NEGATIVE);
  ros2::getParam(node, prefix + "/drag_constant", des.drag_constant, ros2::NON_NEGATIVE);

  ros2::getParam(node, prefix + "/rot_speed_coefs", des.rot_speed_coefs);
  TOBAS_CHECK(des.rot_speed_coefs.first > 0);
  TOBAS_CHECK(des.rot_speed_coefs.second >= 0);

  ros2::getParam(node, prefix + "/channel", des.channel);
}

void Drone::getFixedWingConfig(rclcpp::Node::SharedPtr node)
{
  PRINT_DEBUG("Drone::getFixedWingConfig");

  getVehicleParameters(node);
  getAerodynamicsCoefficients(node);
  getControlSurfaces(node);
}

void Drone::getVehicleParameters(rclcpp::Node::SharedPtr node)
{
  PRINT_DEBUG("Drone::getVehicleParameters");

  const string prefix = "fixed_wing/vehicle";
  auto& des = fixed_wing_.vehicle;

  ros2::getParam(node, prefix + "/wing_surface", des.wing_surface, ros2::POSITIVE);
  ros2::getParam(node, prefix + "/wing_span", des.wing_span, ros2::POSITIVE);
  ros2::getParam(node, prefix + "/mean_aerodynamic_chord", des.mac, ros2::POSITIVE);
  ros2::getParam(node, prefix + "/aerodynamic_center", des.ac.data);
  ros2::getParam(node, prefix + "/alpha_limit/lower", des.alpha_limit.lower);
  ros2::getParam(node, prefix + "/alpha_limit/upper", des.alpha_limit.upper);

  TOBAS_CHECK(des.alpha_limit.isValid());
}

void Drone::getAerodynamicsCoefficients(rclcpp::Node::SharedPtr node)
{
  PRINT_DEBUG("Drone::getAerodynamicsCoefficients");

  const string prefix = "fixed_wing/aerodynamic_coefficients";
  auto& des = fixed_wing_.aerodynamics;

  ros2::getParam(node, prefix + "/c_lift_0", des.c_lift_0, ros2::POSITIVE);
  ros2::getParam(node, prefix + "/c_lift_alpha", des.c_lift_alpha, ros2::POSITIVE);
  ros2::getParam(node, prefix + "/c_drag_0", des.c_drag_0, ros2::POSITIVE);
  ros2::getParam(node, prefix + "/c_drag_alpha", des.c_drag_alpha, ros2::POSITIVE);
  ros2::getParam(node, prefix + "/c_side_beta", des.c_side_beta, ros2::NEGATIVE);

  ros2::getParam(node, prefix + "/c_roll_beta", des.c_roll_beta, ros2::NEGATIVE);
  ros2::getParam(node, prefix + "/c_roll_p", des.c_roll_p, ros2::NEGATIVE);
  ros2::getParam(node, prefix + "/c_roll_r", des.c_roll_r);

  ros2::getParam(node, prefix + "/c_pitch_0", des.c_pitch_0);
  ros2::getParam(node, prefix + "/c_pitch_alpha", des.c_pitch_alpha, ros2::NEGATIVE);
  ros2::getParam(node, prefix + "/c_pitch_abs_beta", des.c_pitch_abs_beta);
  ros2::getParam(node, prefix + "/c_pitch_alpha_rate", des.c_pitch_alpha_rate);
  ros2::getParam(node, prefix + "/c_pitch_q", des.c_pitch_q, ros2::NEGATIVE);

  ros2::getParam(node, prefix + "/c_yaw_beta", des.c_yaw_beta);
  ros2::getParam(node, prefix + "/c_yaw_p", des.c_yaw_p);
  ros2::getParam(node, prefix + "/c_yaw_r", des.c_yaw_r, ros2::NEGATIVE);
}

void Drone::getControlSurfaces(rclcpp::Node::SharedPtr node)
{
  PRINT_DEBUG("Drone::getControlSurfaces");

  // fixed_wing/controll_surface_0などにはnh.searchParam()が使えないため，
  // 制御面の個数を明示的にパラメータサーバから取得する．
  size_t num_cs;
  ros2::getParam(node, "fixed_wing/num_control_surfaces", num_cs);

  fixed_wing_.control_surfaces.resize(num_cs);

  for (size_t cs_idx = 0; cs_idx < num_cs; ++cs_idx)
    getControlSurface(node, cs_idx, fixed_wing_.control_surfaces.at(cs_idx));
}

void Drone::getControlSurface(rclcpp::Node::SharedPtr node, size_t cs_idx, ControlSurface& des)
{
  PRINT_DEBUG("Drone::getControlSurface(" << cs_idx << ")");

  const string prefix = "fixed_wing/control_surface_" + to_string(cs_idx);

  // indexはprefixの番号と同じ
  des.index = cs_idx;

  ros2::getParam(node, prefix + "/angle_limit/lower", des.angle_limit.lower);
  ros2::getParam(node, prefix + "/angle_limit/upper", des.angle_limit.upper);
  TOBAS_CHECK(des.angle_limit.isValid() && des.angle_limit.inRange(0));

  ros2::getParam(node, prefix + "/max_angle_rate", des.max_angle_rate, ros2::POSITIVE);

  ros2::getParam(node, prefix + "/c_lift_delta", des.c_lift_delta);
  ros2::getParam(node, prefix + "/c_drag_abs_delta", des.c_drag_abs_delta);
  ros2::getParam(node, prefix + "/c_side_delta", des.c_side_delta);
  ros2::getParam(node, prefix + "/c_roll_delta", des.c_roll_delta);
  ros2::getParam(node, prefix + "/c_pitch_delta", des.c_pitch_delta);
  ros2::getParam(node, prefix + "/c_yaw_delta", des.c_yaw_delta);
}
}  // namespace tobas

#include "tobas_y_axis_tilt_vtol_controller/transition_handler.hpp"

#include <iostream>

#include <tobas_ros2_tools/time.hpp>

#include "tobas_y_axis_tilt_vtol_controller/constants.hpp"

namespace tobas
{
namespace y_axis_tilt_vtol
{
namespace
{
constexpr double kSafePitchRate = 10.0 / 180.0 * M_PI;  // 仮, [rad/s]
}  // namespace

TransitionHandler::TransitionHandler(TranslationalEoM& _trans_eom) : trans_eom_(_trans_eom)
{
  forward_transition_traj_.setMaxPitchRate(kSafePitchRate);
}

void TransitionHandler::updateCommandType(const CommandType& _command_type)
{
  if (_command_type != last_command_type_) {
    if (_command_type == CommandType::FORWARD_FLIGHT || _command_type == CommandType::BACKWARD_TRANSITION) {
      require_initialization_ = true;
    }
  }
  last_command_type_ = _command_type;
}

void TransitionHandler::setCruiseVelocity(const double& _cruise_velocity)
{
  forward_transition_traj_.setCruiseVelocity(_cruise_velocity);
}

void TransitionHandler::setMinTransitionTime(const double& _time)
{
  const auto max_tilt_velocity = kForwardAngle / _time;
  forward_transition_traj_.setMaxTiltVelocity(max_tilt_velocity);
  backward_transition_traj_.setMaxTiltVelocity(max_tilt_velocity);
}

void TransitionHandler::setAoARange(const st::Range<double>& _alpha_limit)
{
  forward_transition_traj_.setMinAoA(_alpha_limit.lower);
}

void TransitionHandler::setHorizontalNaturalFreq(const double& _freq)
{
  hor_wn_ = _freq;
}

void TransitionHandler::setVerticalNatualFreq(const double& _freq)
{
  ver_wn_ = _freq;
}

void TransitionHandler::solve(
  const tobas_command_msgs::msg::VelRollSideSlipMode::UniquePtr& _cmd,
  const kdl::Vector& _cur_vel_G,
  const kdl::Rotation& _cur_rot,
  double& _roll,
  double& _pitch,
  double& _side_slip,
  double& _thrust,
  double& _thrust_angle)
{
  const auto cur_time = _cmd->header.stamp;
  if (last_command_type_ == CommandType::FORWARD_FLIGHT) {
    if (require_initialization_) {
      // 現在の前進速度から速度のslopeを作る
      const auto [cur_roll, cur_pitch, cur_yaw] = _cur_rot.getRPY();
      const auto cur_vel_YW = calcVelYW(_cur_vel_G, _cur_rot);
      forward_transition_traj_.setStartConditions(cur_time, cur_pitch, _thrust_angle, cur_vel_YW.x());
      require_initialization_ = false;
    }
    _thrust_angle = forward_transition_traj_.calcTargetTiltAngle(cur_time);
    solveForwardFlight(_cmd, _cur_vel_G, _cur_rot, _thrust_angle, _roll, _pitch, _side_slip, _thrust);
  }
  else if (last_command_type_ == CommandType::BACKWARD_TRANSITION) {
    if (require_initialization_) {
      backward_transition_traj_.setStartConditions(cur_time, _thrust_angle);
      require_initialization_ = false;
    }
    _thrust_angle = backward_transition_traj_.calcTargetTiltAngle(cur_time);
    solveBackwardTransition(_cmd, _cur_vel_G, _cur_rot, _thrust_angle, _roll, _pitch, _side_slip, _thrust);
  }
  else {
    std::cerr << "Unexpected command type." << std::endl;
  }
}

void TransitionHandler::solveForwardFlight(
  const tobas_command_msgs::msg::VelRollSideSlipMode::UniquePtr& _cmd,
  const kdl::Vector& _cur_vel_G,
  const kdl::Rotation& _cur_rot,
  const double& _thrust_angle,
  double& _roll,
  double& _pitch,
  double& _side_slip,
  double& _thrust)
{
  // Pass through roll & side slip command
  _roll = _cmd->roll;
  _side_slip = _cmd->side_slip;
  // Compute target acc
  const auto cur_time = _cmd->header.stamp;
  const auto tar_vel_x = forward_transition_traj_.calcTargetVx(cur_time) + _cmd->vel_x;
  const auto cur_vel_YW = calcVelYW(_cur_vel_G, _cur_rot);
  const auto tar_acc_x = hor_wn_ * (tar_vel_x - cur_vel_YW.x()) + forward_transition_traj_.calcTargetAccx(cur_time);
  const auto tar_vel_z = _cmd->vel_z;
  const auto tar_acc_z = ver_wn_ * (tar_vel_z - cur_vel_YW.z());
  // Compute target thrust and pitch.
  // 並進運動の方程式をx, z軸に関して解く tilt角は拘束
  trans_eom_.solveXZDynamics(tar_acc_x, tar_acc_z, _thrust_angle, _cur_rot, cur_vel_YW, _pitch, _thrust);
}

void TransitionHandler::solveBackwardTransition(
  const tobas_command_msgs::msg::VelRollSideSlipMode::UniquePtr& _cmd,
  const kdl::Vector& _cur_vel_G,
  const kdl::Rotation& _cur_rot,
  const double& _thrust_angle,
  double& _roll,
  double& _pitch,
  double& _side_slip,
  double& _thrust)
{
  // Pass through roll & side slip command
  _roll = _cmd->roll;
  _side_slip = _cmd->side_slip;
  // Compute target acc
  const auto cur_vel_YW = calcVelYW(_cur_vel_G, _cur_rot);
  const auto tar_vel_z = _cmd->vel_z;
  const auto tar_acc_z = ver_wn_ * (tar_vel_z - cur_vel_YW.z());
  // Compute target thrust and pitch
  // 並進運動の方程式をz軸だけに関して解く tilt角は拘束
  trans_eom_.solveZDynamics(tar_acc_z, _thrust_angle, _cur_rot, cur_vel_YW, _pitch, _thrust);
}

kdl::Vector TransitionHandler::calcVelYW(const kdl::Vector& _vel_G, const kdl::Rotation& _rot)
{
  const auto [roll, pitch, yaw] = _rot.getRPY();
  const auto vel_YW = kdl::Rotation::RotY(pitch) * kdl::Rotation::RotX(roll) * _vel_G;
  return vel_YW;
}
}  // namespace y_axis_tilt_vtol
}  // namespace tobas

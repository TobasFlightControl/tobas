#include "tobas_y_axis_tilt_vtol_controller/transition_handler.hpp"

#include <iostream>

#include <tobas_ros2_tools/time.hpp>

namespace tobas
{
namespace y_axis_tilt_vtol
{
TransitionHandler::TransitionHandler(TranslationalEoM& trans_eom)
: trans_eom_(trans_eom)
{
}

void TransitionHandler::updateCommandType(
  const CommandType& command_type)
{
  if (command_type != last_command_type_) {
    if (command_type == CommandType::FORWARD_FLIGHT
    || command_type == CommandType::BACKWARD_TRANSITION){
      require_initialization_ = true;
    }
  }
  last_command_type_ = command_type;
}

void TransitionHandler::setCruiseVelocity(const double& cruise_velocity)
{
  forward_transition_traj_.setCruiseVelocity(cruise_velocity);
}

void TransitionHandler::setMaxTiltVelocity(const double& max_tilt_velocity)
{
  forward_transition_traj_.setMaxTiltVelocity(max_tilt_velocity);
  backward_transition_traj_.setMaxTiltVelocity(max_tilt_velocity);
}

void TransitionHandler::setHorizontalNaturalFreq(const double& freq)
{
  hor_wn_ = freq;
}

void TransitionHandler::setVerticalNatualFreq(const double& freq)
{
  ver_wn_ = freq;
}

void TransitionHandler::solve(
  const tobas_command_msgs::msg::VelRollSideSlipMode::UniquePtr& cmd,
  const kdl::Vector& cur_vel_G,
  const kdl::Rotation& cur_rot,
  double& roll,
  double& pitch,
  double& side_slip,
  double& thrust,
  double& thrust_angle)
{
  const auto cur_time = cmd->header.stamp;
  if (last_command_type_ == CommandType::FORWARD_FLIGHT) {
    if (require_initialization_) {
      // 現在の前進速度から速度のslopeを作る
      const auto cur_vel_YW = calcVelYW(cur_vel_G, cur_rot);
      forward_transition_traj_.setStartConditions(cur_time, thrust_angle, cur_vel_YW.x());
      require_initialization_ = false;
    }
    thrust_angle = forward_transition_traj_.calcTargetTiltAngle(cur_time);
    solveForwardFlight(cmd, cur_vel_G, cur_rot, thrust_angle, roll, pitch, side_slip, thrust);
  }
  else if (last_command_type_ == CommandType::BACKWARD_TRANSITION) {
    if (require_initialization_) {
      backward_transition_traj_.setStartConditions(cur_time, thrust_angle);
      require_initialization_ = false;
    }
    thrust_angle = backward_transition_traj_.calcTargetTiltAngle(cur_time);
    solveBackwardTransition(cmd, cur_vel_G, cur_rot, thrust_angle, roll, pitch, side_slip, thrust);
  }
  else {
    std::cerr << "Unexpected command type." << std::endl;
  }
}

void TransitionHandler::solveForwardFlight(
  const tobas_command_msgs::msg::VelRollSideSlipMode::UniquePtr& cmd,
  const kdl::Vector& cur_vel_G,
  const kdl::Rotation& cur_rot,
  const double& thrust_angle,
  double& roll,
  double& pitch,
  double& side_slip,
  double& thrust)
{
  // Pass through roll & side slip command
  roll = cmd->roll;
  side_slip = cmd->side_slip;
  // Compute target acc
  const auto cur_time = cmd->header.stamp;
  const auto tar_vel_x = forward_transition_traj_.calcTargetVx(cur_time) + cmd->vel_x;
  const auto cur_vel_YW = calcVelYW(cur_vel_G, cur_rot);
  const auto tar_acc_x = hor_wn_ * (tar_vel_x - cur_vel_YW.x()) + forward_transition_traj_.calcTargetAccx(cur_time);
  const auto tar_vel_z = cmd->vel_z;
  const auto tar_acc_z = ver_wn_ * (tar_vel_z - cur_vel_YW.z());
  // Compute target thrust and pitch.
  // 並進運動の方程式をx, z軸に関して解く tilt角は拘束
  trans_eom_.solveXZDynamics(
    tar_acc_x,
    tar_acc_z,
    thrust_angle,
    cur_rot,
    cur_vel_YW,
    pitch,
    thrust
  );
}

void TransitionHandler::solveBackwardTransition(
  const tobas_command_msgs::msg::VelRollSideSlipMode::UniquePtr& cmd,
  const kdl::Vector& cur_vel_G,
  const kdl::Rotation& cur_rot,
  const double& thrust_angle,
  double& roll,
  double& pitch,
  double& side_slip,
  double& thrust)
{
  // Pass through roll & side slip command
  roll = cmd->roll;
  side_slip = cmd->side_slip;
  // Compute target acc
  const auto cur_vel_YW = calcVelYW(cur_vel_G, cur_rot);
  const auto tar_vel_z = cmd->vel_z;
  const auto tar_acc_z = ver_wn_ * (tar_vel_z - cur_vel_YW.z());
  // Compute target thrust and pitch
  // 並進運動の方程式をz軸だけに関して解く tilt角は拘束
  trans_eom_.solveZDynamics(
    tar_acc_z,
    thrust_angle,
    cur_rot,
    cur_vel_YW,
    pitch,
    thrust
  );
}

kdl::Vector TransitionHandler::calcVelYW(const kdl::Vector& vel_G, const kdl::Rotation& rot)
{
  const auto [roll, pitch, yaw] = rot.getRPY();
  const auto vel_YW = kdl::Rotation::RotY(pitch) * kdl::Rotation::RotX(roll) * vel_G;
  return vel_YW;
}

} // namespace y_axis_tilt_vtol
} // namespace tobas

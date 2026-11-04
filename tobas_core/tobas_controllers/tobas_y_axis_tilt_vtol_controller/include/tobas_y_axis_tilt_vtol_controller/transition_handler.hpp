#pragma once

#include <cmath>

#include <tobas_kdl/vector.hpp>
#include <tobas_kdl/rotation.hpp>

#include <tobas_command_msgs/msg/vel_roll_side_slip_mode.hpp>

#include "tobas_y_axis_tilt_vtol_controller/backward_transition_trajectory.hpp"
#include "tobas_y_axis_tilt_vtol_controller/forward_transition_trajectory.hpp"
#include "tobas_y_axis_tilt_vtol_controller/translational_eom.hpp"

namespace tobas
{
namespace y_axis_tilt_vtol
{
class TransitionHandler
{
  static constexpr double kUpwardAngle = 0.0; // [rad]
  static constexpr double kForwardAngle = 0.5 * M_PI; // [rad]
public:
  enum CommandType {
    MANUAL_MULTICOPTER,
    MANUAL_FIXED_WING,
    MULTICOPTER,
    FORWARD_FLIGHT,
    BACKWARD_TRANSITION,
  };

  explicit TransitionHandler(TranslationalEoM& trans_eom);

  void updateCommandType(const CommandType& command_type);
  void solve(
    const tobas_command_msgs::msg::VelRollSideSlipMode::UniquePtr& cmd,
    const kdl::Vector& cur_vel_G,
    const kdl::Rotation& cur_rot,
    double& roll,
    double& pitch,
    double& side_slip,
    double& thrust,
    double& thrust_angle);

  void setCruiseVelocity(const double& cruise_velocity); // [m/s]
  void setMaxTiltVelocity(const double& tilt_velocity); // [rad]
  void setHorizontalNaturalFreq(const double& freq); // [rad/s]
  void setVerticalNatualFreq(const double& freq); // [rad/s]

private:
  // mutables
  bool require_initialization_ = false;
  CommandType last_command_type_;
  ForwardTransitionTrajectory forward_transition_traj_;
  BackwardTransitionTrajectory backward_transition_traj_;

  // Config
  TranslationalEoM& trans_eom_;
  double cruise_velocity_;
  double max_tilt_velocity_;
  double hor_wn_;
  double ver_wn_;

  void solveForwardFlight(
    const tobas_command_msgs::msg::VelRollSideSlipMode::UniquePtr& cmd,
    const kdl::Vector& cur_vel_G,
    const kdl::Rotation& cur_rot,
    const double& thrust_angle,
    double& roll,
    double& pitch,
    double& side_slip,
    double& thrust);
  void solveBackwardTransition(
    const tobas_command_msgs::msg::VelRollSideSlipMode::UniquePtr& cmd,
    const kdl::Vector& cur_vel_G,
    const kdl::Rotation& cur_rot,
    const double& thrust_angle,
    double& roll,
    double& pitch,
    double& side_slip,
    double& thrust);
  // calculate velocity viewed in yaw-rotated world frame
  static kdl::Vector calcVelYW(const kdl::Vector& vel_G, const kdl::Rotation& rot);
};
} // namespace y_axis_tilt_vtol
} // namespace tobas

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_control/pid/pi.hpp>
#include <tobas_kdl/tree_fk_solver_pos_all.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>
#include <tobas_kdl/tree_joint_parser.hpp>
#include <tobas_quadprog/dual_active_set.hpp>
#include <tobas_std_tools/range.hpp>
#include <tobas_tools/mixer_i.hpp>

#include "./mixer_solution.hpp"

namespace tobas::y_axis_tilt_vtol
{
/* Thrust mixing for y-axis tilt vtol aircraft using a pseudoinverse matrix. */
class QpMixer : public MixerI
{
  using super = MixerI;

public:
  explicit QpMixer(const Drone& drone, const kdl::Tree& tree);

  void updateInternalDataStructures() override;
  void registerInternalDataReset();
  void setVelocityReliability(const bool& _is_velocity_reliable);

  std::expected<MixerSolution, std::string> solve(
    const double& dt,
    const double& rho,
    const kdl::JntArray& cur_q,
    const kdl::Vector& cur_gyro_B,
    const kdl::Vector& cur_vel_G,
    const kdl::Vector& tar_dgyro_B,
    const double& thrust,
    const double& thrust_angle);

  std::expected<double, std::string> getForwardTiltAngle(size_t idx) const;
  double calcMaxThrust(const double& thrust_angle) const;  ///< thrust_angle [rad] -> max_thrust [N]

private:
  struct Config
  {
    double linear_weight = 1.0;
    double angular_weight = 1.0;
    double thrust_weight = 1e-6;
    double cs_weight = 1e-6;
  } cfg_;

  kdl::TreeJointParser jnt_parser_;
  kdl::TreeFkSolverPosAll fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;

  struct StaticRotorLinkInfo
  {
    bool is_tilt;
    size_t column;
    int sign;  ///< Signs of the Y components of tilt axes viewed from the body frame.
    double min_tilt_angle;
    double max_tilt_angle;
    size_t inequality_row;
  };
  struct DynamicRotorLinkState
  {
    double alpha;  ///< Tilt angle offsets viewed from the body frame [rad].
    double desiable_thrust = 0.0;
    double desiable_thrust_angle = 0.0;
  };
  struct ControlSurfaceInfo
  {
    size_t column;
    size_t inequality_row;
  };
  struct DynamicControlSurfaceState
  {
    double last_angle = 0.0;
  };

  // Fixed values.
  std::vector<StaticRotorLinkInfo> info_;
  std::vector<DynamicRotorLinkState> state_;
  std::vector<ControlSurfaceInfo> cs_info_;
  std::vector<DynamicControlSurfaceState> cs_state_;
  double max_thrust_multicopter_;
  double max_thrust_fixed_wing_;
  double omega_roll_;        ///< クルーズ時ロールモード角周波数
  double omega_s_;           ///< クルーズ時短周期モード角周波数
  double omega_dutch_roll_;  ///< クルーズ時ダッチロールモード角周波数

  // Mutable values.
  quadprog::DualActiveSetSolver
    qp_;                 ///< QP solver. variables : [T_0, T_1x, T_1z, ..., T_n, delta_0, delta_1, ..., delta_N]
  Eigen::Diagonal5d Q_;  ///< EoM weights.
  Eigen::DiagonalXd R_;  ///< Thrust weights.
  Eigen::Matrix5Xd E_;
  Eigen::Vector5d f_;  ///< [M_x, M_y, Mz, u_x, u_z]
  ctrl::PIController x_controller_;
  ctrl::PIController y_controller_;
  ctrl::PIController z_controller_;
  ctrl::PIController z2_controller_;
  bool require_internal_data_reset_ = false;
  bool is_velocity_reliable_ = false;

  void calcModesAngularFreq();
  double
  calcTiltAngle(const double& tx, const double& tz, const StaticRotorLinkInfo& info, const DynamicRotorLinkState& state)
    const;
  double
  calcThrustAngle(const double& tilt_angle, const StaticRotorLinkInfo& info, const DynamicRotorLinkState& state) const;
  void updateControlJointPositions(const kdl::JntArray& cur_q);
  void setDesiableRotorInfo(
    const double& _desiable_thrust,
    const double& _thrust_angle,
    const Eigen::Matrix<double, 5, Eigen::Dynamic>& _E);
  st::Range<Eigen::Vector3d> calcMomentRange(const Eigen::Matrix<double, 5, Eigen::Dynamic>& E) const;
};
}  // namespace tobas::y_axis_tilt_vtol

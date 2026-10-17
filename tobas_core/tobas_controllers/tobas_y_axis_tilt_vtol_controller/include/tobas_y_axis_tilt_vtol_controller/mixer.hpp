// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_control/pid/pi_velocity_form.hpp>
#include <tobas_kdl/tree_fk_solver_pos_all.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>
#include <tobas_tools/mixer_i.hpp>

namespace tobas
{
namespace y_axis_tilt_vtol
{
/* Thrust mixing for y-axis tilt vtol aircraft using a pseudoinverse matrix. */
class Mixer : public MixerI
{
  using super = MixerI;
  static constexpr double kCruiseSpeed = 15.0;  // m/s, 固定翼機運動モード計算のための仮のクルーズ時対気速度

public:
  explicit Mixer(const Drone& drone, const kdl::Tree& tree);

  bool updateInternalDataStructures() override;
  double calcMaxThrust(const double& thrust_angle) const; // thrust_angle [rad] -> max_thrust [N]

  bool solve(
    const double& dt,
    const double& rho,
    const double& airspeed,
    const kdl::JntArray& cur_q,
    const kdl::Vector& cur_gyro_B,
    const kdl::Vector& tar_dgyro_B,
    const double& ux,
    const double& uz);

  double getThrust(size_t idx) const;
  double getTiltAngle(size_t idx) const;
  double getDeflection(size_t idx) const;  // rad

private:
  kdl::TreeFkSolverPosAll fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;

  struct StaticRotorLinkInfo
  {
    bool is_tilt;
    size_t column;
    int sign;  // Signs of the Y components of tilt axes viewed from the body frame.
  };
  struct DynamicRotorLinkState
  {
    double alpha;  // Tilt angle offsets viewed from the body frame [rad].
  };

  // Fixed values.
  std::vector<StaticRotorLinkInfo> info_;
  std::vector<DynamicRotorLinkState> state_;
  size_t cs_start_idx_;
  double max_thrust_multicopter_;
  double max_thrust_fixed_wing_;
  double omega_roll_;        // クルーズ時ロールモード角周波数
  double omega_s_;           // クルーズ時短周期モード角周波数
  double omega_dutch_roll_;  // クルーズ時ダッチロールモード角周波数

  // Mutable values.
  Eigen::Matrix5Xd E_;
  Eigen::Vector5d f_;   // [M_x, M_y, Mz, u_x, u_z]
  Eigen::VectorXd x_;   // e.g. [T_0, T_1x, T_1z, ..., T_n, delta_0, delta_1, ..., delta_N]
  Eigen::VectorXd scale_;
  ctrl::PIVelocityForm x_controller_;
  ctrl::PIVelocityForm y_controller_;
  ctrl::PIVelocityForm z_controller_;
  ctrl::PIVelocityForm z2_controller_;

  bool calcModesAngularFreq();
};
}  // namespace y_axis_tilt_vtol
}  // namespace tobas

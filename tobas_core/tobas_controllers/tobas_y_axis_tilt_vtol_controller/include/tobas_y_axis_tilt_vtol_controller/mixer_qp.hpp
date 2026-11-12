// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

// #include <tobas_control/pid/pi_velocity_form.hpp>
#include <tobas_control/pid/pi.hpp>
#include <tobas_kdl/tree_fk_solver_pos_all.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>
#include <tobas_quadprog/dual_active_set.hpp>
#include <tobas_std_tools/range.hpp>
#include <tobas_tools/mixer_i.hpp>

namespace tobas
{
namespace y_axis_tilt_vtol
{
/* Thrust mixing for y-axis tilt vtol aircraft using a pseudoinverse matrix. */
class QpMixer : public MixerI
{
  using super = MixerI;
  static constexpr size_t kFixedRotorInequalitySize = 2;
  static constexpr size_t kTiltRotorInequalitySize = 4;
  static constexpr size_t kControlSurfaceInequalitySize = 2;
  static constexpr double kCruiseSpeed = 15.0;  // m/s, 固定翼機運動モード計算のための仮のクルーズ時対気速度
  static constexpr double kMomentThrustRatio = 1.0 / 3.0; // 実行可能最大モーメントを出すために行って良い最大推力変更比率 3軸あるので1軸制御分には1/3
  static constexpr double kTiltAngleMargin = 30.0 / 180.0 * M_PI; // 実行可能最大モーメントを出すために行って良い最大ティルト角変更
  // TODO : 機体ごとに定める
  static constexpr double kMaxTiltVelocity = 90.0 / 180.0 * M_PI; // rad/s
  static constexpr double kMinTiltAngle = - 0.25 * M_PI; // rad
  static constexpr double kMaxTiltAngle = 0.75 * M_PI; // rad
  static constexpr double kMaxCsVelocity = M_PI; // rad/s

public:
  explicit QpMixer(const Drone& drone, const kdl::Tree& tree);

  bool updateInternalDataStructures() override;
  double calcMaxThrust(const double& thrust_angle) const; // thrust_angle [rad] -> max_thrust [N]

  bool solve(
    const double& dt,
    const double& rho,
    const kdl::JntArray& cur_q,
    const kdl::Vector& cur_gyro_B,
    const kdl::Vector& cur_vel_G,
    const kdl::Vector& tar_dgyro_B,
    const double& thrust,
    const double& thrust_angle);

  double getThrust(size_t idx) const;
  double getTiltAngle(size_t idx) const;
  double getDeflection(size_t idx) const;  // rad

private:
  struct Config
  {
    double linear_weight = 1.0;
    double angular_weight = 1.0;
    double thrust_weight = 1e-6;
    double cs_weight = 1e-6;
  } cfg_;

  kdl::TreeFkSolverPosAll fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;

  struct StaticRotorLinkInfo
  {
    bool is_tilt;
    size_t column;
    int sign;  // Signs of the Y components of tilt axes viewed from the body frame.
    size_t inequality_row;
  };
  struct DynamicRotorLinkState
  {
    double alpha;  // Tilt angle offsets viewed from the body frame [rad].
    double last_thrust_angle = 0.0;
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
  double omega_roll_;        // クルーズ時ロールモード角周波数
  double omega_s_;           // クルーズ時短周期モード角周波数
  double omega_dutch_roll_;  // クルーズ時ダッチロールモード角周波数

  // Mutable values.
  quadprog::DualActiveSetSolver qp_;  // QP solver. variables : [T_0, T_1x, T_1z, ..., T_n, delta_0, delta_1, ..., delta_N]
  Eigen::Diagonal5d Q_;               // EoM weights.
  Eigen::DiagonalXd R_;               // Thrust weights.
  Eigen::Matrix5Xd E_;
  Eigen::Vector5d f_;   // [M_x, M_y, Mz, u_x, u_z]
  ctrl::PIController x_controller_;
  ctrl::PIController y_controller_;
  ctrl::PIController z_controller_;
  ctrl::PIController z2_controller_;

  bool calcModesAngularFreq();
  double calcTiltAngle(const double& tx, const double& tz, const StaticRotorLinkInfo& info, const DynamicRotorLinkState& state) const;
  st::Range<Eigen::Vector3d> calcMomentRange(
    const double& desiable_thrust,
    const double& thrust_angle,
    const Eigen::Matrix<double, 5, Eigen::Dynamic>& E) const;
};
}  // namespace y_axis_tilt_vtol
}  // namespace tobas

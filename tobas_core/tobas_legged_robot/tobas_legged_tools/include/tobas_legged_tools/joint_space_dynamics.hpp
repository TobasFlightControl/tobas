// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_kdl/euler.hpp>
#include <tobas_kdl/jntspace_inertia_matrix.hpp>
#include <tobas_kdl/tree_bounding_box_solver.hpp>
#include <tobas_kdl/tree_id_solver_rne.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>
#include <tobas_kdl/tree_jacobian_solver.hpp>
#include <tobas_kdl/tree_jntspace_inertia_solver.hpp>
#include <tobas_quadprog/dual_active_set.hpp>
#include <tobas_std_tools/range.hpp>

namespace tobas
{
namespace lr_tools
{
struct JointSpaceDynamicsConfig
{
  double friction_coef;  // [-] Static friction coefficient.
  double foot_diameter;  // [m] Diameter of the foot contact surface.

  double min_normal_force;  // [N]
  double max_normal_force;  // [N]

  double force_weight;  // Penalty for error from the ground reaction force reference.
  double base_weight;   // Penalty for error from the floating-link acceleration reference.
};

/* Compute ground reaction forces and joint torques for realizing floating-link acceleration (memo: 2-70). */
class JointSpaceDynamics
{
  static constexpr size_t kPosIdx = 0;
  static constexpr size_t kYawIdx = 3;
  static constexpr size_t kPitchIdx = 4;
  static constexpr size_t kRollIdx = 5;

  static constexpr size_t kBaseDoF = 6;     // Degrees of freedom of the floating link.
  static constexpr size_t kIneqSize = 8;    // Number of inequality constraints per leg.
  static constexpr size_t kForceSize = 3;   // fx, fy, fz
  static constexpr size_t kTorqueSize = 1;  // tz
  static constexpr size_t kWrenchSize = kForceSize + kTorqueSize;

  static constexpr double kYawAngle = 0.;  // The yaw angle is always zero because the footprint frame is used.
  static constexpr double kStandLegNormalForceThresh =
    1.;  // [N] Treat as a swing leg when the target normal force is at or below this value.

public:
  explicit JointSpaceDynamics(
    const kdl::Tree& tree,
    const std::vector<std::string>& foot_names,
    const std::string& floating_base_name = "");

  bool updateInternalDataStructures();

  bool configure(const JointSpaceDynamicsConfig& cfg);

  bool solve(
    const double& roll,
    const double& pitch,
    const kdl::Vector& cur_vel,
    const kdl::Vector& cur_gyro,
    const kdl::JntArray& cur_q,
    const kdl::JntArray& cur_qd,
    const kdl::Vector& tar_acc,
    const kdl::Euler& tar_rpydd,
    const kdl::JntArray& tar_qdd,
    const std::vector<kdl::Vector>& tar_force,
    const std::vector<double>& tar_torque);

  inline kdl::Vector getFootForce(size_t leg) const;
  inline double getFootTorque(size_t leg) const;
  inline kdl::JntArray getEffort() const;

  inline const std::string& errorMessage() const;

private:
  const kdl::Tree& tree_raw_;
  const std::vector<std::string> foot_names_;
  const std::string floating_base_name_;
  const size_t nc_, wrench_size_;

  // Config
  double friction_coef_;
  st::Range<double> normal_force_range_;

  Eigen::VectorXd w_out_;    // size = 3 * nc_
  Eigen::VectorXd eff_out_;  // size = kBaseDoF + nj_raw_
  std::string error_msg_;

  kdl::Tree tree_;  // Tree with a floating link.
  size_t nj_raw_, nj_;
  kdl::JntArray cur_q_, cur_qd_, tar_qdd_;
  Eigen::MatrixXd J_;                                 // Stacked Jacobians of foot positions.
  Eigen::VectorXd w_ref_;                             // Ground reaction force reference.
  Eigen::Matrix<double, kIneqSize, kWrenchSize> A1_;  // Left-hand side of the inequality matrix equation for each foot.
  Eigen::Matrix<double, kIneqSize, 1> b1_st_,
    b1_sw_;  // Right-hand side of the inequality matrix equation for each foot.
  quadprog::DualActiveSetSolver qp_;

  kdl::TreeJacobianSolver jac_solver_;
  kdl::TreeIdSolver_RNE rne_;
  kdl::TreeJntSpaceInertiaSolver mass_solver_;
  kdl::TreeInertiaSolver inertia_solver_;
  kdl::TreeBoundingBoxSolver bb_solver_;

  double calcMass();
  double calcSizeScale();

  inline size_t forceIndex(const size_t& leg) const;
  inline size_t torqueIndex(const size_t& leg) const;
};

inline kdl::Vector JointSpaceDynamics::getFootForce(size_t leg) const
{
  return kdl::Vector(w_out_.segment<kForceSize>(forceIndex(leg)));
}

inline double JointSpaceDynamics::getFootTorque(size_t leg) const
{
  return w_out_(torqueIndex(leg));
}

inline kdl::JntArray JointSpaceDynamics::getEffort() const
{
  return kdl::JntArray(eff_out_.tail(nj_raw_));
}

inline size_t JointSpaceDynamics::forceIndex(const size_t& leg) const
{
  return kWrenchSize * leg;
}

inline size_t JointSpaceDynamics::torqueIndex(const size_t& leg) const
{
  return kWrenchSize * leg + kForceSize;
}

inline const std::string& JointSpaceDynamics::errorMessage() const
{
  return error_msg_;
}
}  // namespace lr_tools
}  // namespace tobas

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include <tobas_quadprog/dual_active_set.hpp>

#include "./tree_jac_acc_solver.hpp"
#include "./tree_jacobian_solver.hpp"
#include "./tree_joint_parser.hpp"
#include "./tree_solver_i.hpp"

namespace tobas
{
namespace kdl
{
class TreeIkSolverAcc : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeIkSolverAcc(const Tree& tree);

  void updateInternalDataStructures() override;

  /**
   * Calculate inverse acceleration kinematics, from joint positions
   * and cartesian velocities to joint velocities.
   *
   * @param q_in input joint positions
   * @param qd_in input joint velocities
   * @param acc_in input cartesian acceleration
   * @return The solution, or the QP solver error.
   */
  std::expected<JntArray, std::string> cartToJnt(const JntArray& q_in, const JntArray& qd_in, const AccelMap& acc_in);

  const Eigen::Vector6d& getWeightTS() const;
  void setWeightTS(const Eigen::Vector6d& Wt);

  const double& getWeightJS() const;
  void setWeightJS(double Wj);

private:
  TreeJacobianSolver jnt2jac_;
  TreeJacAccSolver jnt2jdqd_;
  TreeJointParser jntparser_;

  Eigen::Vector6d Wt_ = Eigen::Vector6d::Constant(1.0);  ///< Task space weight
  double Wj_ = 1e-3;                                     ///< Joint space weight
  Eigen::VectorXd qdd_min_, qdd_max_;                    ///< Joint acceleration limits
  Eigen::MatrixXd J_;                                    ///< Big jacobian
  Eigen::VectorXd a_;                                    ///< Big acceleration in TS

  quadprog::DualActiveSetSolver qp_solver_;

  void resize();
};
}  // namespace kdl
}  // namespace tobas

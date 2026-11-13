// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include <tobas_quadprog/dual_active_set.hpp>

#include "./tree_jacobian_solver.hpp"
#include "./tree_joint_parser.hpp"
#include "./tree_solver_i.hpp"

namespace tobas::kdl
{
class TreeIkSolverVel : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeIkSolverVel(const Tree& _tree);

  void updateInternalDataStructures() override;

  /**
   * Calculate inverse velocity kinematics, from joint positions and cartesian velocities to joint velocities.
   *
   * @param _q_in input joint positions
   * @param _v_in input cartesian velocity
   * @return The solution, or the QP solver error.
   */
  std::expected<JntArray, std::string> cartToJnt(const JntArray& _q_in, const TwistMap& _v_in);

  const Eigen::Vector6d& getWeightTS() const;
  void setWeightTS(const Eigen::Vector6d& _wt);

  const double& getWeightJS() const;
  void setWeightJS(double _wj);

private:
  TreeJacobianSolver jnt2jac_;
  TreeJointParser jntparser_;

  Eigen::Vector6d Wt_ = Eigen::Vector6d::Constant(1.0);
  double Wj_ = 1e-3;   // TODO: Scale by the supported weight for each joint.
  Eigen::MatrixXd J_;  ///< Big jacobian
  Eigen::VectorXd t_;  ///< Big velocity in TS

  quadprog::DualActiveSetSolver qp_solver_;

  void resize();
};
}  // namespace tobas::kdl

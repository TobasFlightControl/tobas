// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_ik_solver_vel_pinv.hpp"

#include <tobas_eigen_tools/linalg.hpp>
#include <tobas_quadprog/utils.hpp>

using namespace std;
using namespace Eigen;

namespace tobas
{
namespace kdl
{
TreeIkSolverVel_pinv::TreeIkSolverVel_pinv(const Tree& tree) : super(tree), jnt2jac_(tree_), jntparser_(tree_)
{
  resize();
}

bool TreeIkSolverVel_pinv::updateInternalDataStructures()
{
  if (!super::updateInternalDataStructures()) {
    return false;
  }
  if (!jnt2jac_.updateInternalDataStructures()) {
    return false;
  }
  if (!jntparser_.updateInternalDataStructures()) {
    return false;
  }

  resize();

  return true;
}

int TreeIkSolverVel_pinv::cartToJnt(const JntArray& q_in, const TwistMap& v_in)
{
  if (!isUpToDate()) {
    return setDefaultError(kNotUpToDate);
  }
  if (q_in.rows() != nj_) {
    return setDefaultError(kSizeMismatch);
  }

  const auto num_points = v_in.size();
  const auto eq_dim = 6 * num_points;

  // Create big jacobian and velocity
  J_.conservativeResize(eq_dim, nj_);
  t_.conservativeResize(eq_dim);
  size_t i = 0;
  for (const auto& [seg_name, twist] : v_in) {
    // Update big jacobian
    if (jnt2jac_.jntToJac(q_in, seg_name) < 0) {
      return copyError(jnt2jac_);
    }
    J_.block(6 * i, 0, 6, nj_) = jnt2jac_.getJacobian().data;

    // Update big velocity
    t_.segment(6 * i, 3) = twist.vel.data;
    t_.segment(6 * i + 3, 3) = twist.rot.data;

    ++i;
  }

  // Objective function.
  const VectorXd Wt = eigen::tile(Wt_, num_points, 0);
  const VectorXd Wj = VectorXd::Constant(nj_, Wj_);
  const MatrixXd JT_Wt = J_.transpose() * Wt.asDiagonal();
  qp_solver_.problem.P = JT_Wt * J_;
  qp_solver_.problem.P.diagonal() += Wj;
  qp_solver_.problem.q = -JT_Wt * t_;

  // Inequality constraints.
  auto qd_min = -jntparser_.maxVelocities();
  auto qd_max = +jntparser_.maxVelocities();
  for (size_t j = 0; j < nj_; ++j) {
    // If the joint angle limit is already exceeded,
    // constrain the velocity so the violation does not increase further.
    if (q_in(j) < jntparser_.lowerLimit(j)) {
      qd_min(j) = 0.;
    }
    else if (q_in(j) > jntparser_.upperLimit(j)) {
      qd_max(j) = 0.;
    }
  }
  quadprog::matIneqFromRange(qd_min.data, qd_max.data, qp_solver_.problem.A, qp_solver_.problem.b);

  // Solve the QP.
  if (!qp_solver_.solve()) {
    return setDefaultError(kQpFailed);
  }
  qd_out_.data = qp_solver_.solution();

  return setDefaultError(kNoError);
}

bool TreeIkSolverVel_pinv::setWeightTS(const Vector6d& Wt)
{
  if ((Wt.array() < 0).any()) {
    return false;
  }

  Wt_ = Wt;
  return true;
}

const Vector6d& TreeIkSolverVel_pinv::getWeightTS() const
{
  return Wt_;
}

bool TreeIkSolverVel_pinv::setWeightJS(const double& Wj)
{
  // Always include a regularization term to prevent numerical errors.
  if (Wj <= 0) {
    return false;
  }

  Wj_ = Wj;
  return true;
}

const double& TreeIkSolverVel_pinv::getWeightJS() const
{
  return Wj_;
}

void TreeIkSolverVel_pinv::resize()
{
  qp_solver_.x_scale = VectorXd::Ones(nj_);
  qp_solver_.problem.G.conservativeResize(0, nj_);
  qp_solver_.problem.h.conservativeResize(0);
}
}  // namespace kdl
}  // namespace tobas

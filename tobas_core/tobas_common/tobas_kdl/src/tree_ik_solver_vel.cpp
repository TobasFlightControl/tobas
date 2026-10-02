// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_ik_solver_vel.hpp"

#include <cassert>
#include <ranges>
#include <utility>

#include <tobas_eigen_tools/core.hpp>
#include <tobas_quadprog/utils.hpp>

namespace tobas
{
namespace kdl
{
TreeIkSolverVel::TreeIkSolverVel(const Tree& tree) : super(tree), jnt2jac_(tree_), jntparser_(tree_)
{
  resize();
}

void TreeIkSolverVel::updateInternalDataStructures()
{
  jnt2jac_.updateInternalDataStructures();
  jntparser_.updateInternalDataStructures();

  resize();
}

std::expected<JntArray, std::string> TreeIkSolverVel::cartToJnt(const JntArray& q_in, const TwistMap& v_in)
{
  assert(q_in.size() == tree_.getNrOfJoints());

  const auto num_points = v_in.size();
  const auto eq_dim = 6 * num_points;

  // Create big jacobian and velocity.
  J_.conservativeResize(eq_dim, tree_.getNrOfJoints());
  t_.conservativeResize(eq_dim);
  for (const auto& [i, entry] : std::views::enumerate(v_in)) {
    const auto& [seg_name, twist] = entry;

    // Update big jacobian.
    const auto& jacob = jnt2jac_.jntToJac(q_in, seg_name);
    J_.block(6 * i, 0, 6, tree_.getNrOfJoints()) = jacob.data;

    // Update big velocity.
    t_.segment(6 * i, 3) = twist.vel.data;
    t_.segment(6 * i + 3, 3) = twist.rot.data;
  }

  // Objective function.
  const Eigen::VectorXd Wt = eigen::tile(Wt_, num_points, Eigen::Vertical);
  const Eigen::VectorXd Wj = Eigen::VectorXd::Constant(tree_.getNrOfJoints(), Wj_);
  const Eigen::MatrixXd JT_Wt = J_.transpose() * Wt.asDiagonal();
  qp_solver_.problem.P = JT_Wt * J_;
  qp_solver_.problem.P.diagonal() += Wj;
  qp_solver_.problem.q = -JT_Wt * t_;

  // Inequality constraints.
  auto qd_min = -jntparser_.maxVelocities();
  auto qd_max = +jntparser_.maxVelocities();
  for (size_t j = 0; j < tree_.getNrOfJoints(); ++j) {
    // If the joint angle limit is already exceeded,
    // constrain the velocity so the violation does not increase further.
    if (q_in(j) < jntparser_.lowerLimit(j)) {
      qd_min(j) = 0.0;
    }
    else if (q_in(j) > jntparser_.upperLimit(j)) {
      qd_max(j) = 0.0;
    }
  }
  quadprog::matIneqFromRange(qd_min.data, qd_max.data, qp_solver_.problem.A, qp_solver_.problem.b);

  // Solve the QP.
  const auto qd_out = qp_solver_.solve();
  if (!qd_out) {
    return std::unexpected(qd_out.error());
  }
  return JntArray(*qd_out);
}

void TreeIkSolverVel::setWeightTS(const Eigen::Vector6d& Wt)
{
  assert((Wt.array() >= 0.0).all());

  Wt_ = Wt;
}

const Eigen::Vector6d& TreeIkSolverVel::getWeightTS() const
{
  return Wt_;
}

void TreeIkSolverVel::setWeightJS(const double& Wj)
{
  // Always include a regularization term to prevent numerical errors.
  assert(Wj > 0.0);

  Wj_ = Wj;
}

const double& TreeIkSolverVel::getWeightJS() const
{
  return Wj_;
}

void TreeIkSolverVel::resize()
{
  qp_solver_.x_scale = Eigen::VectorXd::Ones(tree_.getNrOfJoints());
  qp_solver_.problem.G.conservativeResize(0, tree_.getNrOfJoints());
  qp_solver_.problem.h.conservativeResize(0);
}
}  // namespace kdl
}  // namespace tobas

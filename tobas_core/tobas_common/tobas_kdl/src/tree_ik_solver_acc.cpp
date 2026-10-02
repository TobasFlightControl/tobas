// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_ik_solver_acc.hpp"

#include <cassert>

#include <tobas_eigen_tools/core.hpp>
#include <tobas_quadprog/utils.hpp>

namespace tobas
{
namespace kdl
{
TreeIkSolverAcc::TreeIkSolverAcc(const Tree& tree) : super(tree), jnt2jac_(tree_), jnt2jdqd_(tree_), jntparser_(tree_)
{
  resize();
}

void TreeIkSolverAcc::updateInternalDataStructures()
{
  jnt2jac_.updateInternalDataStructures();
  jnt2jdqd_.updateInternalDataStructures();
  jntparser_.updateInternalDataStructures();

  resize();
}

std::expected<JntArray, std::string>
TreeIkSolverAcc::cartToJnt(const JntArray& q_in, const JntArray& qd_in, const AccelMap& acc_in)
{
  assert(q_in.size() == tree_.getNrOfJoints());
  assert(qd_in.size() == tree_.getNrOfJoints());

  const auto num_points = acc_in.size();
  const auto eq_dim = 6 * num_points;

  // Update Jdqd.
  const auto& jdqd = jnt2jdqd_.jntToCart(q_in, qd_in);

  // Create big jacobian and acceleration.
  J_.conservativeResize(eq_dim, tree_.getNrOfJoints());
  a_.conservativeResize(eq_dim);
  size_t i = 0;
  for (const auto& [seg_name, accel] : acc_in) {
    // Update big jacobian.
    const auto& jacob = jnt2jac_.jntToJac(q_in, seg_name);
    J_.block(6 * i, 0, 6, tree_.getNrOfJoints()) = jacob.data;

    // Update big acceleration.
    const auto& Jdqd = jdqd.at(seg_name);
    a_.segment(6 * i, 3) = (accel.linear - Jdqd.linear).data;
    a_.segment(6 * i + 3, 3) = (accel.angular - Jdqd.angular).data;

    ++i;
  }

  // Objective function.
  const Eigen::VectorXd Wt = eigen::tile(Wt_, num_points, Eigen::Vertical);
  const Eigen::VectorXd Wj = Eigen::VectorXd::Constant(tree_.getNrOfJoints(), Wj_);
  const Eigen::MatrixXd JT_Wt = J_.transpose() * Wt.asDiagonal();
  qp_solver_.problem.P = JT_Wt * J_;
  qp_solver_.problem.P.diagonal() += Wj;
  qp_solver_.problem.q = -JT_Wt * a_;

  // Inequality constraints.
  qdd_min_.fill(-INFINITY);
  qdd_max_.fill(INFINITY);
  for (size_t j = 0; j < tree_.getNrOfJoints(); ++j) {
    // If the joint angle limit is already exceeded,
    // constrain the acceleration so the violation does not increase further.
    if (q_in(j) < jntparser_.lowerLimit(j)) {
      qdd_min_(j) = 0.0;
    }
    else if (q_in(j) > jntparser_.upperLimit(j)) {
      qdd_max_(j) = 0.0;
    }
  }
  quadprog::matIneqFromRange(qdd_min_, qdd_max_, qp_solver_.problem.A, qp_solver_.problem.b);

  // Solve the QP.
  const auto qdd_out = qp_solver_.solve();
  if (!qdd_out) {
    return std::unexpected(qdd_out.error());
  }
  return JntArray(*qdd_out);
}

void TreeIkSolverAcc::setWeightTS(const Eigen::Vector6d& Wt)
{
  assert((Wt.array() >= 0.0).all());

  Wt_ = Wt;
}

const Eigen::Vector6d& TreeIkSolverAcc::getWeightTS() const
{
  return Wt_;
}

void TreeIkSolverAcc::setWeightJS(const double& Wj)
{
  // Always include a regularization term to prevent numerical errors.
  assert(Wj > 0.0);

  Wj_ = Wj;
}

const double& TreeIkSolverAcc::getWeightJS() const
{
  return Wj_;
}

void TreeIkSolverAcc::resize()
{
  qdd_min_.conservativeResize(tree_.getNrOfJoints());
  qdd_max_.conservativeResize(tree_.getNrOfJoints());

  qp_solver_.x_scale = Eigen::VectorXd::Ones(tree_.getNrOfJoints());
  qp_solver_.problem.G.conservativeResize(0, tree_.getNrOfJoints());
  qp_solver_.problem.h.conservativeResize(0);
}
}  // namespace kdl
}  // namespace tobas

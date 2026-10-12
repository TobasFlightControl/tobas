// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_jacobian_solver.hpp"

#include <cassert>

namespace tobas
{
namespace kdl
{
TreeJacobianSolver::TreeJacobianSolver(const Tree& tree) : super(tree)
{
  resize();
}

void TreeJacobianSolver::updateInternalDataStructures()
{
  resize();
}

const Jacobian& TreeJacobianSolver::jntToJac(const JntArray& q_in, const std::string& seg_name)
{
  assert(q_in.rows() == tree_.getNrOfJoints());
  assert(tree_.hasSegment(seg_name));

  // Lets recursively iterate until we are in the root segment.
  const auto root_it = tree_.getRootSegment();
  auto cur_it = tree_.getSegment(seg_name);
  auto T_total = Frame::Identity();

  while (cur_it != root_it) {
    const auto& cur_elem = cur_it->second;
    const auto& cur_seg = cur_elem.segment;
    const auto& j = cur_elem.q_nr;
    const auto& qj = q_in(j);

    // Get the pose of the segment.
    const auto T_local = cur_seg.pose(qj);
    // Calculate new T_end.
    T_total = T_local * T_total;

    // Get the twist of the segment.
    if (cur_seg.joint().type != Joint::kFixed) {
      auto t_local = cur_seg.jacobian(qj);
      // Transform the endpoint of the local twist to the global endpoint.
      t_local = t_local.refPoint(T_total.p - T_local.p);
      // Transform the base of the twist to the endpoint.
      t_local = T_total.M.inverse(t_local);
      // Store the twist in the jacobian.
      jac_out_.setColumn(j, t_local);
    }

    // Go to the parent.
    cur_it = cur_elem.parent;
  }

  // Change the base of the complete jacobian from the endpoint to the base.
  jac_out_.changeBase(T_total.M);

  return jac_out_;
}

void TreeJacobianSolver::resize()
{
  jac_out_ = Jacobian::Zero(tree_.getNrOfJoints());
}
}  // namespace kdl
}  // namespace tobas

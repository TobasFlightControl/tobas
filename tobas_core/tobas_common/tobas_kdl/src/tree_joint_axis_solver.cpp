// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_joint_axis_solver.hpp"

#include <cassert>

namespace tobas
{
namespace kdl
{
TreeJointAxisSolver::TreeJointAxisSolver(const Tree& tree) : super(tree), fk_solver_(tree)
{
}

void TreeJointAxisSolver::updateInternalDataStructures()
{
  fk_solver_.updateInternalDataStructures();
}

Vector TreeJointAxisSolver::jntToCart(const JntArray& q_in, const std::string& seg_name)
{
  assert(q_in.size() == tree_.getNrOfJoints());

  const auto cur_it = tree_.getSegment(seg_name);
  assert(cur_it != tree_.getSegments().end());
  assert(cur_it != tree_.getRootSegment());

  const auto& cur_ele = cur_it->second;
  const auto& cur_jnt = cur_ele.segment.joint();
  const auto& par_name = cur_ele.parent->first;

  const auto par_frame = fk_solver_.jntToCart(q_in, par_name);
  return par_frame.M * cur_jnt.axis();
}
}  // namespace kdl
}  // namespace tobas

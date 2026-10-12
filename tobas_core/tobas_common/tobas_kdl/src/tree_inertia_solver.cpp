// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_inertia_solver.hpp"

#include <cassert>

namespace tobas
{
namespace kdl
{
TreeInertiaSolver::TreeInertiaSolver(const Tree& tree) : super(tree)
{
}

RigidBodyInertia TreeInertiaSolver::jntToCart(const JntArray& q)
{
  assert(q.size() == tree_.getNrOfJoints());

  const auto root_it = tree_.getRootSegment();
  return jntToCartRec(root_it, q);
}

RigidBodyInertia TreeInertiaSolver::jntToCartRec(const SegmentMap::const_iterator& cur_it, const JntArray& q)
{
  const auto& cur_elem = cur_it->second;
  const auto& cur_seg = cur_elem.segment;

  auto inertia = cur_seg.inertia();
  for (const auto& child_it : cur_elem.children) {
    const auto& child_elem = child_it->second;
    const auto& child_seg = child_elem.segment;
    const auto qj = child_seg.joint().type == Joint::kFixed ? 0.0 : q(child_elem.q_nr);
    inertia += child_seg.pose(qj) * jntToCartRec(child_it, q);
  }

  return inertia;
}
}  // namespace kdl
}  // namespace tobas

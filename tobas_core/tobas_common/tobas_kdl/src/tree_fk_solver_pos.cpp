// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_fk_solver_pos.hpp"

#include <cassert>

namespace tobas
{
namespace kdl
{
TreeFkSolverPos::TreeFkSolverPos(const Tree& tree) : super(tree)
{
}

Frame TreeFkSolverPos::jntToCart(const JntArray& q, const std::string& seg_name)
{
  assert(q.size() == tree_.getNrOfJoints());
  assert(tree_.hasSegment(seg_name));

  const auto seg_it = tree_.getSegment(seg_name);
  return recursiveFk(q, seg_it);
}

Frame TreeFkSolverPos::recursiveFk(const JntArray& q, const SegmentMap::const_iterator& cur_it)
{
  const auto& cur_ele = cur_it->second;
  const auto& cur_seg = cur_ele.segment;
  const auto& cur_idx = cur_ele.q_nr;
  const auto& cur_frame = cur_seg.pose(q(cur_idx));

  if (cur_it == tree_.getRootSegment()) {
    return cur_frame;
  }

  const auto& parent_it = cur_ele.parent;
  return recursiveFk(q, parent_it) * cur_frame;
}
}  // namespace kdl
}  // namespace tobas

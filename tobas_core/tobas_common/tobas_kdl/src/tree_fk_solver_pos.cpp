// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_fk_solver_pos.hpp"

#include <cassert>

namespace tobas::kdl
{
TreeFkSolverPos::TreeFkSolverPos(const Tree& _tree) : super(_tree)
{
}

Frame TreeFkSolverPos::jntToCart(const JntArray& _q, const std::string& _seg_name)
{
  assert(_q.size() == tree_.getNrOfJoints());
  assert(tree_.hasSegment(_seg_name));

  const auto seg_it = tree_.getSegment(_seg_name);
  return recursiveFk(_q, seg_it);
}

Frame TreeFkSolverPos::recursiveFk(const JntArray& _q, const SegmentMap::const_iterator& _cur_it)
{
  const auto& cur_ele = _cur_it->second;
  const auto& cur_seg = cur_ele.segment;
  const auto& cur_idx = cur_ele.q_nr;
  const auto& cur_frame = cur_seg.pose(_q(cur_idx));

  if (_cur_it == tree_.getRootSegment()) {
    return cur_frame;
  }

  const auto& parent_it = cur_ele.parent;
  return recursiveFk(_q, parent_it) * cur_frame;
}
}  // namespace tobas::kdl

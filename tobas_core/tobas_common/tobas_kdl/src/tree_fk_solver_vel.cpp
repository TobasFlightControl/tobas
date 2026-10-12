// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_fk_solver_vel.hpp"

#include <cassert>

namespace tobas
{
namespace kdl
{
TreeFkSolverVel::TreeFkSolverVel(const Tree& _tree) : super(_tree)
{
}

FrameVel TreeFkSolverVel::jntToCart(const JntArray& _q, const JntArray& _qd, const std::string& _seg_name)
{
  assert(_q.size() == tree_.getNrOfJoints());
  assert(_qd.size() == tree_.getNrOfJoints());
  assert(tree_.hasSegment(_seg_name));

  const auto seg_it = tree_.getSegment(_seg_name);
  return recursiveFk(_q, _qd, seg_it);
}

FrameVel TreeFkSolverVel::recursiveFk(const JntArray& _q, const JntArray& _qd, const SegmentMap::const_iterator& _cur_it)
{
  const auto& cur_ele = _cur_it->second;
  const auto& cur_seg = cur_ele.segment;
  const auto& cur_idx = cur_ele.q_nr;

  const auto pose = cur_seg.pose(_q(cur_idx));
  const auto twist = cur_seg.twist(_q(cur_idx), _qd(cur_idx));
  const FrameVel cur_framevel(pose, twist);

  if (_cur_it == tree_.getRootSegment()) {
    return cur_framevel;
  }

  const auto& parent_it = cur_ele.parent;
  return recursiveFk(_q, _qd, parent_it) * cur_framevel;
}
}  // namespace kdl
}  // namespace tobas

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_fk_solver_vel.hpp"

#include <cassert>

namespace tobas
{
namespace kdl
{
TreeFkSolverVel::TreeFkSolverVel(const Tree& tree) : super(tree)
{
}

FrameVel TreeFkSolverVel::jntToCart(const JntArray& q, const JntArray& qd, const std::string& seg_name)
{
  assert(q.size() == tree_.getNrOfJoints());
  assert(qd.size() == tree_.getNrOfJoints());
  assert(tree_.hasSegment(seg_name));

  const auto seg_it = tree_.getSegment(seg_name);
  return recursiveFk(q, qd, seg_it);
}

FrameVel TreeFkSolverVel::recursiveFk(const JntArray& q, const JntArray& qd, const SegmentMap::const_iterator& cur_it)
{
  const auto& cur_ele = cur_it->second;
  const auto& cur_seg = cur_ele.segment;
  const auto& cur_idx = cur_ele.q_nr;

  const auto pose = cur_seg.pose(q(cur_idx));
  const auto twist = cur_seg.twist(q(cur_idx), qd(cur_idx));
  const FrameVel cur_framevel(pose, twist);

  if (cur_it == tree_.getRootSegment()) {
    return cur_framevel;
  }

  const auto& parent_it = cur_ele.parent;
  return recursiveFk(q, qd, parent_it) * cur_framevel;
}
}  // namespace kdl
}  // namespace tobas

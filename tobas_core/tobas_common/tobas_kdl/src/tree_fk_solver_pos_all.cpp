// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_fk_solver_pos_all.hpp"

#include <cassert>

namespace tobas::kdl
{
TreeFkSolverPosAll::TreeFkSolverPosAll(const Tree& _tree) : super(_tree)
{
  updateInternalDataStructures();
}

void TreeFkSolverPosAll::updateInternalDataStructures()
{
  frames_out_.clear();
  for (const auto& [name, _] : tree_.getSegments()) {
    frames_out_.emplace(name, Frame::Identity());
  }
}

const FrameMap& TreeFkSolverPosAll::jntToCart(const JntArray& _q)
{
  assert(_q.size() == tree_.getNrOfJoints());

  const auto root_it = tree_.getRootSegment();
  const auto& root_name = root_it->first;
  const auto& root_ele = root_it->second;

  auto& root_frame = frames_out_.at(root_name);
  root_frame.setIdentity();

  for (const auto& child_it : root_ele.children) {
    recursiveFk(_q, root_frame, child_it);
  }

  return frames_out_;
}

void TreeFkSolverPosAll::recursiveFk(
  const JntArray& _q,
  const Frame& _par_frame,
  const SegmentMap::const_iterator& _cur_it)
{
  // Get the current segment.
  const auto& cur_name = _cur_it->first;
  const auto& cur_ele = _cur_it->second;

  // Fill the frame for the current segment.
  frames_out_.at(cur_name) = _par_frame * cur_ele.segment.pose(_q(cur_ele.q_nr));

  // Spread to the children.
  for (const auto& child_it : cur_ele.children) {
    recursiveFk(_q, frames_out_.at(cur_name), child_it);
  }
}
}  // namespace tobas::kdl

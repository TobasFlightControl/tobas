// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_fk_solver_pos_all.hpp"

#include <cassert>

namespace tobas
{
namespace kdl
{
TreeFkSolverPosAll::TreeFkSolverPosAll(const Tree& tree) : super(tree)
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

const FrameMap& TreeFkSolverPosAll::jntToCart(const JntArray& q)
{
  assert(q.size() == tree_.getNrOfJoints());

  const auto root_it = tree_.getRootSegment();
  const auto& root_name = root_it->first;
  const auto& root_ele = root_it->second;

  auto& root_frame = frames_out_.at(root_name);
  root_frame.setIdentity();

  for (const auto& child_it : root_ele.children) {
    recursiveFk(q, root_frame, child_it);
  }

  return frames_out_;
}

void TreeFkSolverPosAll::recursiveFk(const JntArray& q, const Frame& par_frame, const SegmentMap::const_iterator& cur_it)
{
  // Get the current segment.
  const auto& cur_name = cur_it->first;
  const auto& cur_ele = cur_it->second;

  // Fill the frame for the current segment.
  frames_out_.at(cur_name) = par_frame * cur_ele.segment.pose(q(cur_ele.q_nr));

  // Spread to the children.
  for (const auto& child_it : cur_ele.children) {
    recursiveFk(q, frames_out_.at(cur_name), child_it);
  }
}
}  // namespace kdl
}  // namespace tobas

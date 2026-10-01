// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./frames.hpp"
#include "./jntarray.hpp"
#include "./tree_solver_i.hpp"

namespace tobas
{
namespace kdl
{
/** Compute the positions of all frames at once. */
class TreeFkSolverPosAll : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeFkSolverPosAll(const Tree& tree);

  void updateInternalDataStructures() override;

  const FrameMap& jntToCart(const JntArray& q);

private:
  FrameMap frames_out_;

  void recursiveFk(const JntArray& q, const Frame& par_frame, const SegmentMap::const_iterator& cur_it);
};
}  // namespace kdl
}  // namespace tobas

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

#include "./frame_vel.hpp"
#include "./jntarray.hpp"
#include "./tree_solver_i.hpp"

namespace tobas
{
namespace kdl
{
class TreeFkSolverVel : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeFkSolverVel(const Tree& tree);

  FrameVel jntToCart(const JntArray& q, const JntArray& qd, const std::string& seg_name);

private:
  FrameVel recursiveFk(const JntArray& q, const JntArray& qd, const SegmentMap::const_iterator& seg_it);
};
}  // namespace kdl
}  // namespace tobas

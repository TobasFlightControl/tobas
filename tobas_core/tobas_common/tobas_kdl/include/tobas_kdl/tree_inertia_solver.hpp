// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./jntarray.hpp"
#include "./rigid_body_inertia.hpp"
#include "./tree_solver_i.hpp"

namespace tobas
{
namespace kdl
{
class TreeInertiaSolver : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeInertiaSolver(const Tree& tree);

  /** Compute mass properties around the root link. */
  RigidBodyInertia jntToCart(const JntArray& q);

private:
  RigidBodyInertia jntToCartRec(const SegmentMap::const_iterator& cur_it, const JntArray& q);
};
}  // namespace kdl
}  // namespace tobas

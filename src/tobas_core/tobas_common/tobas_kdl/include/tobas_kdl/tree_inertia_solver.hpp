// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./jntarray.hpp"
#include "./rigid_body_inertia.hpp"
#include "./tree_solver_i.hpp"

namespace tobas::kdl
{
class TreeInertiaSolver : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeInertiaSolver(const Tree& _tree);

  /** Compute mass properties around the root link. */
  RigidBodyInertia jntToCart(const JntArray& _q);

private:
  RigidBodyInertia jntToCartRec(const SegmentMap::const_iterator& _cur_it, const JntArray& _q);
};
}  // namespace tobas::kdl

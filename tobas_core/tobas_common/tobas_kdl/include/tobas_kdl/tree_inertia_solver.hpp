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

  bool updateInternalDataStructures() override;

  /* Compute mass properties around the root link. */
  int jntToCart(const JntArray& q);

  inline const RigidBodyInertia& getInertia() const;

private:
  std::map<std::string, Frame> X_;
  std::map<std::string, RigidBodyInertia> I_;

  void initialize();
  void step(const SegmentMap::const_iterator& cur_it, const JntArray& q);
};

inline const RigidBodyInertia& TreeInertiaSolver::getInertia() const
{
  return I_.at(tree_.getRootSegment()->first);
}
}  // namespace kdl
}  // namespace tobas

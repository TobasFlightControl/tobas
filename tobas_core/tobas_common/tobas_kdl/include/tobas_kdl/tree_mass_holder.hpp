// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./tree_solver_i.hpp"

namespace tobas::kdl
{
/** Hold the total mass of the tree. */
class TreeMassHolder : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeMassHolder(const Tree& _tree);

  void updateInternalDataStructures() override;

  inline const double& getMass() const;

private:
  double mass_;

  void updateTotalMass();

  /** Return the mass under the specified segment. */
  double computeMass(const SegmentMap::const_iterator& _cur_it) const;
};

inline const double& TreeMassHolder::getMass() const
{
  return mass_;
}
}  // namespace tobas::kdl

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./tree_solver_i.hpp"

namespace tobas
{
namespace kdl
{
/* Hold only the total mass of the `Tree`. */
class TreeMassHolder : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeMassHolder(const Tree& tree);

  bool updateInternalDataStructures() override;

  inline const double& getMass() const;

private:
  double mass_;

  void updateTotalMass();

  /* Return the mass under the specified segment. */
  double computeMass(const SegmentMap::const_iterator& cur_it);
};

inline const double& TreeMassHolder::getMass() const
{
  return mass_;
}
}  // namespace kdl
}  // namespace tobas

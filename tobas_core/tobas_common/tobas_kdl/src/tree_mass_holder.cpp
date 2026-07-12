// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_mass_holder.hpp"

#include <iostream>

#include <tobas_algorithm/kahan.hpp>

using namespace std;

namespace tobas
{
namespace kdl
{
TreeMassHolder::TreeMassHolder(const Tree& tree) : super(tree)
{
  updateTotalMass();
}

bool TreeMassHolder::updateInternalDataStructures()
{
  if (!super::updateInternalDataStructures()) {
    return false;
  }

  updateTotalMass();
  return true;
}

void TreeMassHolder::updateTotalMass()
{
  mass_ = computeMass(tree_.getRootSegment());
}

double TreeMassHolder::computeMass(const SegmentMap::const_iterator& cur_it)
{
  const auto& cur_ele = cur_it->second;
  const auto& cur_seg = cur_ele.segment;

  algo::Kahan<double> mass_sum;

  // Compute the total mass of the child segments.
  for (const auto& child_it : cur_ele.children) {
    mass_sum.add(computeMass(child_it));
  }

  // To reduce numerical error, add the current segment's mass last because segments
  // near the root tend to have greater mass.
  mass_sum.add(cur_seg.inertia().getMass());

  return mass_sum.get();
}
}  // namespace kdl
}  // namespace tobas

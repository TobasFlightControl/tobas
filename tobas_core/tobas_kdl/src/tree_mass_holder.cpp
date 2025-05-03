#include <iostream>

#include <tobas_algorithm/kahan.hpp>

#include "../include/tobas_kdl/tree_mass_holder.hpp"

using namespace std;

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

  // Compute the total mass of child segments
  for (const auto& child_it : cur_ele.children) {
    mass_sum.add(computeMass(child_it));
  }

  // To reduce numerical errors, the mass of the current segment,
  // which is nearest to the root and tend to be large, is added last.
  mass_sum.add(cur_seg.inertia().getMass());

  return mass_sum.get();
}
}  // namespace kdl

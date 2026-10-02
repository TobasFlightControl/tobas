// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_jntspace_inertia_solver.hpp"

#include <cassert>

namespace tobas
{
namespace kdl
{
TreeJntSpaceInertiaSolver::TreeJntSpaceInertiaSolver(const Tree& tree)
  : super(tree), rne_bias_(tree_, kdl::Vector::Zero()), rne_mass_(tree_, kdl::Vector::Zero())
{
  resize();
}

void TreeJntSpaceInertiaSolver::updateInternalDataStructures()
{
  rne_bias_.updateInternalDataStructures();
  rne_mass_.updateInternalDataStructures();

  resize();
}

const JntSpaceInertiaMatrix& TreeJntSpaceInertiaSolver::jntToMass(const JntArray& q)
{
  assert(q.size() == nj_);

  const auto& bias = rne_bias_.cartToJnt(q, q_zero_, q_zero_);

  for (size_t i = 0; i < nj_; ++i) {
    const auto& mass = rne_mass_.cartToJnt(q, q_zero_, elements_[i]);
    mass_out_.data.col(i) = mass.data - bias.data;
  }

  return mass_out_;
}

void TreeJntSpaceInertiaSolver::resize()
{
  nj_ = tree_.getNrOfJoints();

  elements_.assign(nj_, JntArray::Zero(nj_));
  for (size_t i = 0; i < nj_; ++i) {
    elements_[i](i) = 1.0;
  }

  q_zero_ = JntArray::Zero(nj_);
  mass_out_.resize(nj_);
}
}  // namespace kdl
}  // namespace tobas

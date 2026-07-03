// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_jntspace_inertia_solver.hpp"

using namespace std;

namespace tobas
{
namespace kdl
{
TreeJntSpaceInertiaSolver::TreeJntSpaceInertiaSolver(const Tree& tree) : super(tree), rne_(tree_, kdl::Vector::Zero())
{
  resize();
}

bool TreeJntSpaceInertiaSolver::updateInternalDataStructures()
{
  if (!super::updateInternalDataStructures()) {
    return false;
  }

  if (!rne_.updateInternalDataStructures()) {
    return false;
  }

  resize();

  return true;
}

int TreeJntSpaceInertiaSolver::jntToMass(const JntArray& q)
{
  if (!isUpToDate()) {
    return setDefaultError(kNotUpToDate);
  }
  if (q.rows() != nj_ || jntarray_null_.rows() != nj_) {
    return setDefaultError(kSizeMismatch);
  }

  if (rne_.cartToJnt(q, jntarray_null_, jntarray_null_) < 0) {
    return copyError(rne_);
  }
  const auto bias = rne_.getEfforts();  // Copy because the value is overwritten next.

  for (size_t i = 0; i < nj_; ++i) {
    if (rne_.cartToJnt(q, jntarray_null_, elements_[i]) < 0) {
      return copyError(rne_);
    }
    const auto m = rne_.getEfforts() - bias;
    H_out_.data.col(i) = m.data;
  }

  return setDefaultError(kNoError);
}

void TreeJntSpaceInertiaSolver::resize()
{
  elements_.resize(nj_, JntArray::Zero(nj_));
  for (size_t i = 0; i < nj_; ++i) {
    elements_[i](i) = 1;
  }

  H_out_.resize(nj_);
  jntarray_null_ = JntArray::Zero(nj_);
}
}  // namespace kdl
}  // namespace tobas

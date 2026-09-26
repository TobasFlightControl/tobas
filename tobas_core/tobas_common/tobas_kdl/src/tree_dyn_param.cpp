// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_dyn_param.hpp"

namespace tobas
{
namespace kdl
{
TreeDynParam::TreeDynParam(const Tree& tree, const Vector& grav)
  : super(tree), rne_coriolis_(tree_, kdl::Vector::Zero()), rne_gravity_(tree_, grav)
{
  resize();
}

void TreeDynParam::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  rne_coriolis_.updateInternalDataStructures();
  rne_gravity_.updateInternalDataStructures();

  resize();
}

int TreeDynParam::jntToCoriolis(const JntArray& q, const JntArray& qd)
{
  rne_coriolis_.cartToJnt(q, qd, jntarray_null_);
  return copyError(rne_coriolis_);
}

int TreeDynParam::jntToGravity(const JntArray& q)
{
  rne_gravity_.cartToJnt(q, jntarray_null_, jntarray_null_);
  return copyError(rne_gravity_);
}

void TreeDynParam::resize()
{
  jntarray_null_ = JntArray::Zero(nj_);
}
}  // namespace kdl
}  // namespace tobas

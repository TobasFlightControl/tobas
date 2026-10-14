// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_dyn_param.hpp"

namespace tobas::kdl
{
TreeDynParam::TreeDynParam(const Tree& _tree, const Vector& _grav)
  : super(_tree), rne_coriolis_(tree_, kdl::Vector::Zero()), rne_gravity_(tree_, _grav)
{
  resize();
}

void TreeDynParam::updateInternalDataStructures()
{
  rne_coriolis_.updateInternalDataStructures();
  rne_gravity_.updateInternalDataStructures();

  resize();
}

const JntArray& TreeDynParam::jntToCoriolis(const JntArray& _q, const JntArray& _qd)
{
  return rne_coriolis_.cartToJnt(_q, _qd, q_zero_);
}

const JntArray& TreeDynParam::jntToGravity(const JntArray& _q)
{
  return rne_gravity_.cartToJnt(_q, q_zero_, q_zero_);
}

void TreeDynParam::resize()
{
  q_zero_ = JntArray::Zero(tree_.getNrOfJoints());
}
}  // namespace tobas::kdl

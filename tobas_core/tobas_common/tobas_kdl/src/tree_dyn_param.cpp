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
  rne_coriolis_.updateInternalDataStructures();
  rne_gravity_.updateInternalDataStructures();

  resize();
}

const JntArray& TreeDynParam::jntToCoriolis(const JntArray& q, const JntArray& qd)
{
  return rne_coriolis_.cartToJnt(q, qd, q_zero_);
}

const JntArray& TreeDynParam::jntToGravity(const JntArray& q)
{
  return rne_gravity_.cartToJnt(q, q_zero_, q_zero_);
}

void TreeDynParam::resize()
{
  q_zero_ = JntArray::Zero(tree_.getNrOfJoints());
}
}  // namespace kdl
}  // namespace tobas

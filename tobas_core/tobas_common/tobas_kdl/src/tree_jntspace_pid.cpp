// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_jntspace_pid.hpp"

#include <cassert>

namespace tobas
{
namespace kdl
{
TreeJntSpacePID::TreeJntSpacePID(const Tree& tree, const Vector& grav) : super(tree), rne_(tree, grav)
{
  resize();
}

void TreeJntSpacePID::updateInternalDataStructures()
{
  rne_.updateInternalDataStructures();

  resize();
}

const JntArray& TreeJntSpacePID::cartToJnt(
  const JntArray& cur_q,
  const JntArray& cur_qd,
  const JntArray& tar_q,
  const JntArray& tar_qd,
  const JntArray& qdd_ff)
{
  assert(cur_q.size() == tree_.getNrOfJoints());
  assert(cur_qd.size() == tree_.getNrOfJoints());
  assert(tar_q.size() == tree_.getNrOfJoints());
  assert(tar_qd.size() == tree_.getNrOfJoints());
  assert(qdd_ff.size() == tree_.getNrOfJoints());

  // Compute target joint accelerations.
  // TODO: Add the I term.
  const auto tar_qdd = qdd_ff + kp_ * (tar_q - cur_q) + kd_ * (tar_qd - cur_qd);

  // Compute target joint efforts.
  return rne_.cartToJnt(cur_q, cur_qd, tar_qdd);
}

const JntArray&
TreeJntSpacePID::cartToJnt(const JntArray& cur_q, const JntArray& cur_qd, const JntArray& tar_q, const JntArray& tar_qd)
{
  return cartToJnt(cur_q, cur_qd, tar_q, tar_qd, q_zero_);
}

void TreeJntSpacePID::setStiffness(double kp)
{
  assert(kp >= 0.0);
  kp_ = kp;
}

void TreeJntSpacePID::setDamping(double kd)
{
  assert(kd >= 0.0);
  kd_ = kd;
}

void TreeJntSpacePID::resize()
{
  q_zero_ = JntArray::Zero(tree_.getNrOfJoints());
}
}  // namespace kdl
}  // namespace tobas

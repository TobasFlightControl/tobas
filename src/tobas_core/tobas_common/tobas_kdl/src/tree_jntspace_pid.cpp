// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_jntspace_pid.hpp"

#include <cassert>

namespace tobas::kdl
{
TreeJntSpacePID::TreeJntSpacePID(const Tree& _tree, const Vector& _grav) : super(_tree), rne_(_tree, _grav)
{
  resize();
}

void TreeJntSpacePID::updateInternalDataStructures()
{
  rne_.updateInternalDataStructures();

  resize();
}

const JntArray& TreeJntSpacePID::cartToJnt(
  const JntArray& _cur_q,
  const JntArray& _cur_qd,
  const JntArray& _tar_q,
  const JntArray& _tar_qd,
  const JntArray& _qdd_ff)
{
  assert(_cur_q.size() == tree_.getNrOfJoints());
  assert(_cur_qd.size() == tree_.getNrOfJoints());
  assert(_tar_q.size() == tree_.getNrOfJoints());
  assert(_tar_qd.size() == tree_.getNrOfJoints());
  assert(_qdd_ff.size() == tree_.getNrOfJoints());

  // Compute target joint accelerations.
  // TODO: Add the I term.
  const auto tar_qdd = _qdd_ff + kp_ * (_tar_q - _cur_q) + kd_ * (_tar_qd - _cur_qd);

  // Compute target joint efforts.
  return rne_.cartToJnt(_cur_q, _cur_qd, tar_qdd);
}

const JntArray& TreeJntSpacePID::cartToJnt(
  const JntArray& _cur_q,
  const JntArray& _cur_qd,
  const JntArray& _tar_q,
  const JntArray& _tar_qd)
{
  return cartToJnt(_cur_q, _cur_qd, _tar_q, _tar_qd, q_zero_);
}

void TreeJntSpacePID::setStiffness(double _kp)
{
  assert(_kp >= 0.0);
  kp_ = _kp;
}

void TreeJntSpacePID::setDamping(double _kd)
{
  assert(_kd >= 0.0);
  kd_ = _kd;
}

void TreeJntSpacePID::resize()
{
  q_zero_ = JntArray::Zero(tree_.getNrOfJoints());
}
}  // namespace tobas::kdl

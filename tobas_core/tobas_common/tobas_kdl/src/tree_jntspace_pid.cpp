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
  super::updateInternalDataStructures();

  rne_.updateInternalDataStructures();

  resize();
}

int TreeJntSpacePID::cartToJnt(
  const JntArray& cur_q,
  const JntArray& cur_qd,
  const JntArray& tar_q,
  const JntArray& tar_qd,
  const JntArray& qdd_ff)
{
  if (!isUpToDate()) {
    return setDefaultError(kNotUpToDate);
  }
  if (cur_q.rows() != nj_ || cur_qd.rows() != nj_ || tar_q.rows() != nj_ || tar_qd.rows() != nj_ || qdd_ff.rows() != nj_) {
    return setDefaultError(kSizeMismatch);
  }

  // Compute target joint accelerations.
  // TODO: Add the I term.
  const auto tar_qdd = qdd_ff + kp_ * (tar_q - cur_q) + kd_ * (tar_qd - cur_qd);

  // Compute target joint efforts.
  if (rne_.cartToJnt(cur_q, cur_qd, tar_qdd) < 0) {
    return copyError(rne_);
  }

  return setDefaultError(kNoError);
}

int TreeJntSpacePID::cartToJnt(
  const JntArray& cur_q,
  const JntArray& cur_qd,
  const JntArray& tar_q,
  const JntArray& tar_qd)
{
  return cartToJnt(cur_q, cur_qd, tar_q, tar_qd, zeros_);
}

void TreeJntSpacePID::setStiffness(const double& kp)
{
  assert(kp >= 0.0);
  kp_ = kp;
}

void TreeJntSpacePID::setDamping(const double& kd)
{
  assert(kd >= 0.0);
  kd_ = kd;
}

void TreeJntSpacePID::resize()
{
  zeros_ = JntArray::Zero(nj_);
}
}  // namespace kdl
}  // namespace tobas

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_jntspace_pid.hpp"

using namespace std;

namespace tobas
{
namespace kdl
{
TreeJntSpacePID::TreeJntSpacePID(const Tree& tree, const Vector& grav) : super(tree), rne_(tree, grav)
{
  resize();
}

bool TreeJntSpacePID::updateInternalDataStructures()
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

  // Compute target joint accelerations
  // TODO: Add the I term.
  const auto tar_qdd = qdd_ff + kp_ * (tar_q - cur_q) + kd_ * (tar_qd - cur_qd);

  // Compute target joint efforts
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

bool TreeJntSpacePID::setStiffness(const double& kp)
{
  if (kp < 0) {
    return false;
  }

  kp_ = kp;
  return true;
}

bool TreeJntSpacePID::setDamping(const double& kd)
{
  if (kd < 0) {
    return false;
  }

  kd_ = kd;
  return true;
}

void TreeJntSpacePID::resize()
{
  zeros_ = JntArray::Zero(nj_);
}
}  // namespace kdl
}  // namespace tobas

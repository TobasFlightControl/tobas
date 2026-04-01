// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/chain_jacobian_solver.hpp"

using namespace std;

namespace tobas
{
namespace kdl
{
ChainJacobianSolver::ChainJacobianSolver(const Chain& chain) : super(chain)
{
  resize();
}

bool ChainJacobianSolver::updateInternalDataStructures()
{
  if (!super::updateInternalDataStructures()) {
    return false;
  }

  resize();

  return true;
}

bool ChainJacobianSolver::setLockedJoints(const vector<bool> locked_joints)
{
  if (locked_joints.size() != locked_joints_.size()) {
    return false;
  }

  locked_joints_ = locked_joints;
  return true;
}

int ChainJacobianSolver::jntToJac(const JntArray& q_in, int _seg_nr)
{
  if (!isUpToDate()) {
    return setDefaultError(kNotUpToDate);
  }
  if (q_in.rows() != nj_) {
    return setDefaultError(kSizeMismatch);
  }

  const size_t seg_nr = _seg_nr >= 0 ? _seg_nr : chain_.getNrOfSegments();
  if (seg_nr > chain_.getNrOfSegments()) {
    return setDefaultError(kOutputRange);
  }

  // Initialize Jacobian to zero since only seg_nr columns are computed
  J_out_.setZero();

  T_tmp_.setIdentity();
  j_ = k_ = 0;
  for (size_t i = 0; i < seg_nr; ++i) {
    const auto& seg = chain_.getSegment(i);

    // Calculate new Frame_base_ee
    const auto qj = seg.joint().type != Joint::kFixed ? q_in(j_) : 0.;
    const auto T_total = T_tmp_ * seg.pose(qj);  // pose of the new end-point expressed in the base

    // Changing Refpoint of all columns to new ee
    J_out_.changeRefPoint(T_total.p - T_tmp_.p);

    // Only increase jointnr if the segment has a joint
    if (seg.joint().type != Joint::kFixed) {
      // Only put the twist inside if it is not locked
      if (!locked_joints_[j_]) {
        J_out_.setColumn(k_++, T_tmp_.M * seg.jacobian(q_in(j_)));
      }
      ++j_;
    }

    T_tmp_ = T_total;
  }
  return setDefaultError(kNoError);
}

void ChainJacobianSolver::resize()
{
  locked_joints_.resize(nj_, false);
  J_out_.resize(nj_);
}
}  // namespace kdl
}  // namespace tobas

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_jac_acc_solver.hpp"

#include <cassert>

namespace tobas
{
namespace kdl
{
TreeJacAccSolver::TreeJacAccSolver(const Tree& tree) : super(tree)
{
  resize();
}

void TreeJacAccSolver::updateInternalDataStructures()
{
  resize();
}

const AccelMap& TreeJacAccSolver::jntToCart(const JntArray& q, const JntArray& qd)
{
  assert(q.size() == tree_.getNrOfJoints());
  assert(qd.size() == tree_.getNrOfJoints());

  jntToCartRec(tree_.getRootSegment(), q, qd);
  return jdqd_out_;
}

void TreeJacAccSolver::resize()
{
  jdqd_out_.clear();
  R_.clear();
  v_.clear();
  a_.clear();

  for (const auto& [seg_name, _] : tree_.getSegments()) {
    jdqd_out_.emplace(seg_name, Accel::Zero());
    R_[seg_name] = Rotation::Identity();
    v_[seg_name] = Twist::Zero();
    a_[seg_name] = Accel::Zero();
  }
}

void TreeJacAccSolver::jntToCartRec(const SegmentMap::const_iterator& segment, const JntArray& q, const JntArray& qd)
{
  const auto& seg = segment->second.segment;
  const auto& seg_name = segment->first;

  // Do forward calculations.
  double qj, qdj;
  if (seg.joint().type != Joint::kFixed) {
    const auto& j = segment->second.q_nr;
    qj = q(j);
    qdj = qd(j);
  }
  else {
    qj = qdj = 0.0;
  }

  const auto Xj = seg.pose(qj);
  const auto vj = Xj.M.inverse(seg.twist(qj, qdj));  // Transform velocity.

  if (segment == tree_.getRootSegment()) {
    R_.at(seg_name) = Rotation::Identity();
    v_.at(seg_name) = vj;
    a_.at(seg_name) = vj * vj;
  }
  else {
    const auto& par_name = segment->second.parent->first;
    R_.at(seg_name) = R_.at(par_name) * Xj.M;
    v_.at(seg_name) = Xj.inverse(v_.at(par_name)) + vj;
    a_.at(seg_name) = Xj.inverse(a_.at(par_name)) + v_.at(seg_name) * vj;
  }

  // Calculate Jdqd wrt. the root frame.
  jdqd_out_.at(seg_name) = R_.at(seg_name) * a_.at(seg_name);

  // propagate calculations over each child segment.
  for (const auto& child : segment->second.children) {
    jntToCartRec(child, q, qd);
  }
}
}  // namespace kdl
}  // namespace tobas

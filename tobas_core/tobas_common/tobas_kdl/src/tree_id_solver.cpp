// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/tree_id_solver.hpp"

#include <cassert>

namespace tobas
{
namespace kdl
{
TreeIdSolver::TreeIdSolver(const Tree& tree, const Vector& grav) : super(tree), ag_(-grav, Vector::Zero())
{
  resize();
}

void TreeIdSolver::updateInternalDataStructures()
{
  resize();
}

const JntArray&
TreeIdSolver::cartToJnt(const JntArray& q, const JntArray& qd, const JntArray& qdd, const WrenchMap& f_ext)
{
  assert(q.size() == tree_.getNrOfJoints());
  assert(qd.size() == tree_.getNrOfJoints());
  assert(qdd.size() == tree_.getNrOfJoints());

  // Do the recursion here.
  rneStep(tree_.getRootSegment(), q, qd, qdd, f_ext);

  return effort_out_;
}

void TreeIdSolver::resize()
{
  for (const auto& [cur_name, _] : tree_.getSegments()) {
    v_[cur_name] = Twist::Zero();
    a_[cur_name] = Accel::Zero();
    f_[cur_name] = Wrench::Zero();
  }

  effort_out_ = JntArray::Zero(tree_.getNrOfJoints());
}

void TreeIdSolver::rneStep(
  const SegmentMap::const_iterator& cur_it,
  const JntArray& q,
  const JntArray& qd,
  const JntArray& qdd,
  const WrenchMap& f_ext)
{
  const auto& cur_name = cur_it->first;
  const auto& cur_ele = cur_it->second;
  const auto& cur_seg = cur_ele.segment;
  const auto& par_it = cur_ele.parent;
  const auto& par_name = cur_it == tree_.getRootSegment() ? cur_name : par_it->first;

  // Do forward calculations involving velocity & acceleration of this segment.
  const auto& j = cur_ele.q_nr;
  double qj, qdj, qddj;
  if (cur_seg.joint().type != Joint::kFixed) {
    qj = q(j);
    qdj = qd(j);
    qddj = qdd(j);
  }
  else {
    qj = 0.0;
    qdj = 0.0;
    qddj = 0.0;
  }

  const auto Xj = cur_seg.pose(qj);
  const auto Sj = Xj.M.inverse(cur_seg.jacobian(qj));    // Jacobian for current joint
  const auto vj = Xj.M.inverse(cur_seg.twist(qj, qdj));  // Transform velocity

  // Calculate velocity and acceleration of the segment (in segment coordinates).
  if (cur_it == tree_.getRootSegment()) {
    v_.at(cur_name) = vj;
    a_.at(cur_name) = Xj.inverse(ag_) + Sj.accel(qddj) + vj * vj;
  }
  else {
    v_.at(cur_name) = Xj.inverse(v_.at(par_name)) + vj;
    a_.at(cur_name) = Xj.inverse(a_.at(par_name)) + Sj.accel(qddj) + v_.at(cur_name) * vj;
  }

  // Calculate the force for the joint.
  // Collect RigidBodyInertia and external forces.
  const auto& I = cur_seg.inertia();
  f_.at(cur_name) = I * a_.at(cur_name) + v_.at(cur_name) * (I * v_.at(cur_name));
  if (f_ext.find(cur_name) != f_ext.end()) {
    f_.at(cur_name) = f_.at(cur_name) - f_ext.at(cur_name);
  }

  // propagate calculations over each child segment.
  for (const auto& child : cur_ele.children) {
    rneStep(child, q, qd, qdd, f_ext);
  }

  // Do backward calculations involving wrenches and joint efforts.
  // If there is a moving joint, evaluate its effort.
  if (cur_seg.joint().type != Joint::kFixed) {
    effort_out_(j) = Sj.dot(f_.at(cur_name));
    // TODO: Should inertia, damping, and friction be compensated?
  }

  // Add reaction forces to parent segment.
  if (cur_it != tree_.getRootSegment()) {
    f_.at(par_name) = f_.at(par_name) + Xj * f_.at(cur_name);
  }
}
}  // namespace kdl
}  // namespace tobas

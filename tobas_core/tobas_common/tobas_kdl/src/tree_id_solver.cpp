// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/treeidsolver_recursive_newton_euler.cpp
// Modified by Tobas, Inc.

// Copyright  (C)  2009  Ruben Smits <ruben dot smits at intermodalics dot eu>

// Version: 1.0
// Author: Franco Fusco <franco dot fusco at ls2n dot fr>
// Maintainer: Ruben Smits <ruben dot smits at intermodalics dot eu>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.

// You should have received a copy of the GNU General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include "tobas_kdl/tree_id_solver.hpp"

#include <cassert>

namespace tobas::kdl
{
TreeIdSolver::TreeIdSolver(const Tree& _tree, const Vector& _grav) : super(_tree), ag_(-_grav, Vector::Zero())
{
  resize();
}

void TreeIdSolver::updateInternalDataStructures()
{
  resize();
}

const JntArray&
TreeIdSolver::cartToJnt(const JntArray& _q, const JntArray& _qd, const JntArray& _qdd, const WrenchMap& _f_ext)
{
  assert(_q.size() == tree_.getNrOfJoints());
  assert(_qd.size() == tree_.getNrOfJoints());
  assert(_qdd.size() == tree_.getNrOfJoints());

  // Do the recursion here.
  rneStep(tree_.getRootSegment(), _q, _qd, _qdd, _f_ext);

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
  const SegmentMap::const_iterator& _cur_it,
  const JntArray& _q,
  const JntArray& _qd,
  const JntArray& _qdd,
  const WrenchMap& _f_ext)
{
  const auto& cur_name = _cur_it->first;
  const auto& cur_ele = _cur_it->second;
  const auto& cur_seg = cur_ele.segment;
  const auto& par_it = cur_ele.parent;
  const auto& par_name = _cur_it == tree_.getRootSegment() ? cur_name : par_it->first;

  // Do forward calculations involving velocity & acceleration of this segment.
  const auto& j = cur_ele.q_nr;
  double qj, qdj, qddj;
  if (cur_seg.joint().type != Joint::kFixed) {
    qj = _q(j);
    qdj = _qd(j);
    qddj = _qdd(j);
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
  if (_cur_it == tree_.getRootSegment()) {
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
  if (_f_ext.find(cur_name) != _f_ext.end()) {
    f_.at(cur_name) = f_.at(cur_name) - _f_ext.at(cur_name);
  }

  // propagate calculations over each child segment.
  for (const auto& child : cur_ele.children) {
    rneStep(child, _q, _qd, _qdd, _f_ext);
  }

  // Do backward calculations involving wrenches and joint efforts.
  // If there is a moving joint, evaluate its effort.
  if (cur_seg.joint().type != Joint::kFixed) {
    effort_out_(j) = Sj.dot(f_.at(cur_name));
    // TODO: Should inertia, damping, and friction be compensated?
  }

  // Add reaction forces to parent segment.
  if (_cur_it != tree_.getRootSegment()) {
    f_.at(par_name) = f_.at(par_name) + Xj * f_.at(cur_name);
  }
}
}  // namespace tobas::kdl

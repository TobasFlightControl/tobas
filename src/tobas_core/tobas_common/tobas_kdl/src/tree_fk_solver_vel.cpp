// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/chainfksolvervel_recursive.cpp
// Modified by Tobas, Inc.

// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
// Copyright  (C)  2007  Ruben Smits <ruben dot smits at intermodalics dot eu>

// Version: 1.0
// Author: Francois Cauwe <francois at cauwe dot org>
// Author: Ruben Smits <ruben dot smits at intermodalics dot eu>
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

#include "tobas_kdl/tree_fk_solver_vel.hpp"

#include <cassert>

namespace tobas::kdl
{
TreeFkSolverVel::TreeFkSolverVel(const Tree& _tree) : super(_tree)
{
}

FrameVel TreeFkSolverVel::jntToCart(const JntArray& _q, const JntArray& _qd, const std::string& _seg_name)
{
  assert(_q.size() == tree_.getNrOfJoints());
  assert(_qd.size() == tree_.getNrOfJoints());
  assert(tree_.hasSegment(_seg_name));

  const auto seg_it = tree_.getSegment(_seg_name);
  return recursiveFk(_q, _qd, seg_it);
}

FrameVel TreeFkSolverVel::recursiveFk(const JntArray& _q, const JntArray& _qd, const SegmentMap::const_iterator& _cur_it)
{
  const auto& cur_ele = _cur_it->second;
  const auto& cur_seg = cur_ele.segment;
  const auto& cur_idx = cur_ele.q_nr;

  const auto pose = cur_seg.pose(_q(cur_idx));
  const auto twist = cur_seg.twist(_q(cur_idx), _qd(cur_idx));
  const FrameVel cur_framevel(pose, twist);

  if (_cur_it == tree_.getRootSegment()) {
    return cur_framevel;
  }

  const auto& parent_it = cur_ele.parent;
  return recursiveFk(_q, _qd, parent_it) * cur_framevel;
}
}  // namespace tobas::kdl

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/treeidsolver_recursive_newton_euler.hpp
// Modified by Tobas, Inc.

// Copyright  (C)  2009  Ruben Smits <ruben dot smits at intermodalics dot eu>

// Version: 1.0
// Author: Franco Fusco <franco dot fusco at ls2n dot fr>
// Maintainer: Ruben Smits <ruben dot smits at intermodalics dot eu>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#pragma once

#include <tobas_std_tools/universal_constants.hpp>

#include "./jntarray.hpp"
#include "./tree_solver_i.hpp"

namespace tobas::kdl
{
/**
 * Recursive newton euler inverse dynamics solver for kinematic trees.
 *
 * It calculates the torques for the joints, given the motion of the joints (q,qd,qdd),
 * external forces on the segments (expressed in the segments reference frame)
 * and the dynamical parameters of the segments.
 *
 * This is an extension of the inverse dynamic solver for kinematic chains,
 * see `ChainIdSolver_RNE`. The main difference is the use of STL maps
 * instead of vectors to represent external wrenches
 * (as well as internal variables exploited during the recursion).
 */
class TreeIdSolver : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeIdSolver(const Tree& _tree, const Vector& _grav = Vector(0, 0, -st::kGravity));

  void updateInternalDataStructures() override;

  /**
   * Calculate inverse dynamics, from joint positions, velocity, acceleration, external forces
   * to joint torques/forces.
   *
   * @param _q input joint positions
   * @param _qd input joint velocities
   * @param _qdd input joint accelerations
   * @param _f_ext the external forces (no gravity) on the segments
   * @return The solution.
   */
  const JntArray&
  cartToJnt(const JntArray& _q, const JntArray& _qd, const JntArray& _qdd, const WrenchMap& _f_ext = WrenchMap());

private:
  const Accel ag_;

  TwistMap v_;
  AccelMap a_;
  WrenchMap f_;

  JntArray effort_out_;

  void resize();

  void rneStep(
    const SegmentMap::const_iterator& _cur_it,
    const JntArray& _q,
    const JntArray& _qd,
    const JntArray& _qdd,
    const WrenchMap& _f_ext);
};
}  // namespace tobas::kdl

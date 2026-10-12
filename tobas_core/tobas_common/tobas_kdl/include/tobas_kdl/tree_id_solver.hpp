// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_std_tools/universal_constants.hpp>

#include "./jntarray.hpp"
#include "./tree_solver_i.hpp"

namespace tobas
{
namespace kdl
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
  explicit TreeIdSolver(const Tree& tree, const Vector& grav = Vector(0, 0, -st::kGravity));

  void updateInternalDataStructures() override;

  /**
   * Calculate inverse dynamics, from joint positions, velocity, acceleration, external forces
   * to joint torques/forces.
   *
   * @param q input joint positions
   * @param qd input joint velocities
   * @param qdd input joint accelerations
   * @param f_ext the external forces (no gravity) on the segments
   * @return The solution.
   */
  const JntArray&
  cartToJnt(const JntArray& q, const JntArray& qd, const JntArray& qdd, const WrenchMap& f_ext = WrenchMap());

private:
  const Accel ag_;

  TwistMap v_;
  AccelMap a_;
  WrenchMap f_;

  JntArray effort_out_;

  void resize();

  void rneStep(
    const SegmentMap::const_iterator& cur_it,
    const JntArray& q,
    const JntArray& qd,
    const JntArray& qdd,
    const WrenchMap& f_ext);
};
}  // namespace kdl
}  // namespace tobas

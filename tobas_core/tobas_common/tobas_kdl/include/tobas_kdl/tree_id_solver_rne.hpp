// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_std_tools/universal_constants.hpp>

#include "./tree_id_solver.hpp"

namespace tobas
{
namespace kdl
{
/**
 * @brief Recursive newton euler inverse dynamics solver for kinematic trees.
 *
 * It calculates the torques for the joints, given the motion of the joints (q,qdot,qdotdot),
 * external forces on the segments (expressed in the segments reference frame)
 * and the dynamical parameters of the segments.
 *
 * This is an extension of the inverse dynamic solver for kinematic chains,
 * see `ChainIdSolver_RNE`. The main difference is the use of STL maps
 * instead of vectors to represent external wrenches
 * (as well as internal variables exploited during the recursion).
 */
class TreeIdSolver_RNE : public TreeIdSolver
{
  using super = TreeIdSolver;

public:
  explicit TreeIdSolver_RNE(const Tree& tree, const Vector& grav = Vector(0, 0, -st::kGravity));

  bool updateInternalDataStructures() override;

  /**
   * @brief Function to calculate from Cartesian forces to joint torques.
   *
   * @param q The current joint positions
   * @param qd The current joint velocities
   * @param qdd The current joint accelerations
   * @param f_ext The external forces (no gravity) on the segments
   */
  int cartToJnt(const JntArray& q, const JntArray& qd, const JntArray& qdd, const WrenchMap& f_ext = WrenchMap())
    override;

private:
  const Accel ag_;
  TwistMap v_;
  AccelMap a_;
  WrenchMap f_;
  double qj_, qdj_, qddj_;

  void initialize();

  void rneStep(
    const SegmentMap::const_iterator& cur_it,
    const JntArray& q,
    const JntArray& qd,
    const JntArray& qdd,
    const WrenchMap& f_ext);
};
}  // namespace kdl
}  // namespace tobas

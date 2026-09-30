// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./jntarray.hpp"
#include "./tree_solver_i.hpp"

namespace tobas
{
namespace kdl
{
/**
 * @brief Compute the `Jd qd` term in `xdd = J qd + Jd qd`.
 * `Jd qd` can be obtained by running the forward propagation of RNE with `qdd = 0` and `grav = 0`.
 *
 * cf. `tree_id_solver.cpp`
 */
class TreeJacAccSolver : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeJacAccSolver(const Tree& tree);

  void updateInternalDataStructures() override;

  const AccelMap& jntToCart(const JntArray& q, const JntArray& qd);

private:
  AccelMap jdqd_out_;

  RotationMap R_;
  TwistMap v_;
  AccelMap a_;

  void resize();
  void jntToCartRec(const SegmentMap::const_iterator& segment, const JntArray& q, const JntArray& qd);
};

}  // namespace kdl
}  // namespace tobas

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./chain_ik_solver.hpp"
#include "./chain_jacobian_solver.hpp"

namespace tobas
{
namespace kdl
{
/* Official `ChainIkSolverVel_pinv` extended to also support translation-only velocity. */
class ChainIkSolverVel_pinv : public ChainIkSolverVel
{
  using super = ChainIkSolverVel;

public:
  explicit ChainIkSolverVel_pinv(const Chain& chain);

  bool updateInternalDataStructures() override;

  int cartToJnt(const JntArray& q, const Vector& v) override;
  int cartToJnt(const JntArray& q, const Twist& v) override;

private:
  ChainJacobianSolver jnt2jac_;
};
}  // namespace kdl
}  // namespace tobas

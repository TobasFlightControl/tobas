// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./chain_id_solver_rne.hpp"

namespace tobas
{
namespace kdl
{
/* Based on the official `ChainDynParam`, with gravitational acceleration provided at call time. */
class ChainDynParam : public ChainSolverI
{
  using super = ChainSolverI;

public:
  explicit ChainDynParam(const Chain& chain);

  bool updateInternalDataStructures() override;

  int jntToCoriolis(const JntArray& q, const JntArray& qd);
  int jntToGravity(const JntArray& q, const Vector& grav);

  inline const JntArray& getCoriolisEffort() const;
  inline const JntArray& getGravityEffort() const;

private:
  ChainIdSolver_RNE rne_coriolis_;
  ChainIdSolver_RNE rne_gravity_;

  JntArray zero_jntarray_;
  Wrenches zero_wrenches_;
  const Vector zero_vector_ = Vector::Zero();

  void resize();
};

inline const JntArray& ChainDynParam::getCoriolisEffort() const
{
  return rne_coriolis_.getEfforts();
}

inline const JntArray& ChainDynParam::getGravityEffort() const
{
  return rne_gravity_.getEfforts();
}
}  // namespace kdl
}  // namespace tobas

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./chain_solver_i.hpp"
#include "./frames.hpp"
#include "./jntarray.hpp"

namespace tobas
{
namespace kdl
{
/* Based on the official `ChainIdSolver_RNE`, with gravitational acceleration provided at call time. */
class ChainIdSolver_RNE : public ChainSolverI
{
  using super = ChainSolverI;

public:
  explicit ChainIdSolver_RNE(const Chain& chain);

  bool updateInternalDataStructures() override;

  int cartToJnt(
    const JntArray& q,
    const JntArray& qd,
    const JntArray& qdd,
    const Wrenches& forces,  // Forces on each frame, expressed from each frame
    const Vector& grav);

  int cartToJnt(
    const JntArray& q,
    const JntArray& qd,
    const JntArray& qdd,
    const Wrench& f_ee,  // Force on the EE, expressed from the base
    const Vector& grav);

  inline const JntArray& getEfforts() const;

private:
  std::vector<Frame> X_;
  std::vector<SegmentJacobian> S_;
  std::vector<Twist> v_;
  std::vector<Accel> a_;
  std::vector<Wrench> f_;

  int j_;
  double qj_, qdj_, qddj_;
  Wrench f_ee_;
  Accel ag_;

  JntArray effort_out_;

  void resize();
};

inline const JntArray& ChainIdSolver_RNE::getEfforts() const
{
  return effort_out_;
}
}  // namespace kdl
}  // namespace tobas

// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./chain_solver_i.hpp"
#include "./jntarray.hpp"

namespace tobas
{
namespace kdl
{
/**
 * @brief Compute the `Jd qd` term in `xdd = J qd + Jd qd`.
 * `Jd qd` can be obtained by running the forward propagation of RNE with `qdd = 0` and `grav = 0`.
 *
 * cf. `chain_id_solver_rne.cpp`
 */
class ChainJacAccSolver : public ChainSolverI
{
  using super = ChainSolverI;

public:
  explicit ChainJacAccSolver(const Chain& chain);

  bool updateInternalDataStructures() override;

  int jntToCart(const JntArray& q, const JntArray& qd);

  inline const Accel& getJdqd() const;

private:
  std::vector<Frame> X_;
  std::vector<Twist> v_;
  std::vector<Accel> a_;
  Accel Jdqd_out_;
  size_t j_;
  double qj_, qdj_;

  void resize();
};

inline const Accel& ChainJacAccSolver::getJdqd() const
{
  return Jdqd_out_;
}
}  // namespace kdl
}  // namespace tobas

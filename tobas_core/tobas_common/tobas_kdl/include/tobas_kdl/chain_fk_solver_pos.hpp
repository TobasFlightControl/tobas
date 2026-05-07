// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./chain_solver_i.hpp"
#include "./frame.hpp"
#include "./jntarray.hpp"

namespace tobas
{
namespace kdl
{
/**
 * @brief Implementation of a recursive forward position kinematics
 * algorithm to calculate the position transformation from joint
 * space to Cartesian space of a general kinematic chain (kdl::Chain).
 */
class ChainFkSolverPos : public ChainSolverI
{
  using super = ChainSolverI;

public:
  explicit ChainFkSolverPos(const Chain& chain);

  /* Calculate forward position kinematics for a kdl::Chain, from joint coordinates to cartesian pose. */
  int jntToCart(const JntArray& q_in, int seg_nr = -1);

  inline const Frame& getFrame() const;

private:
  size_t j_;

  Frame p_out_;
};

inline const Frame& ChainFkSolverPos::getFrame() const
{
  return p_out_;
}
}  // namespace kdl
}  // namespace tobas

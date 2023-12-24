#pragma once

#include "./chainfksolver.hpp"

namespace KDL
{
/**
 * Implementation of a recursive forward position and velocity
 * kinematics algorithm to calculate the position and velocity
 * transformation from joint space to Cartesian space of a general
 * kinematic chain (KDL::Chain).
 */
class ChainFkSolverVel_recursive : public ChainFkSolverVel
{
  using super = ChainFkSolverVel;

public:
  explicit ChainFkSolverVel_recursive(const Chain& chain);

  void updateInternalDataStructures() override;

  int JntToCart(const JntArrayVel& q_in, int seg_nr = -1) override;
};
}  // namespace KDL

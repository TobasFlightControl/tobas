#pragma once

#include "./chainfksolver.hpp"

namespace kdl
{
/**
 * Implementation of a recursive forward position and velocity
 * kinematics algorithm to calculate the position and velocity
 * transformation from joint space to Cartesian space of a general
 * kinematic chain (kdl::Chain).
 */
class ChainFkSolverVel_recursive : public ChainFkSolverVel
{
  using super = ChainFkSolverVel;

public:
  explicit ChainFkSolverVel_recursive(const Chain& chain);

  void updateInternalDataStructures() override;

  int JntToCart(const JntArray& q_in, const JntArray& qd_in, int seg_nr = -1) override;

private:
  size_t j_;
  double qj_, qdj_;
};
}  // namespace kdl

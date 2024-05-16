#pragma once

#include "./chainfksolver.hpp"

namespace tobas_kdl
{
/**
 * Implementation of a recursive forward position kinematics
 * algorithm to calculate the position transformation from joint
 * space to Cartesian space of a general kinematic chain (tobas_kdl::Chain).
 *
 * @ingroup KinematicFamily
 */
class ChainFkSolverPos_recursive : public ChainFkSolverPos
{
  using super = ChainFkSolverPos;

public:
  explicit ChainFkSolverPos_recursive(const Chain& chain);

  void updateInternalDataStructures() override;

  int JntToCart(const JntArray& q_in, int seg_nr = -1) override;
};
}  // namespace tobas_kdl

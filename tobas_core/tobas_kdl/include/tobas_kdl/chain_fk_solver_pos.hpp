#pragma once

#include "./chain_solver_i.hpp"
#include "./frame.hpp"
#include "./jntarray.hpp"

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
  int JntToCart(const JntArray& q_in, int seg_nr = -1);

  const Frame& getFrame() const
  {
    return p_out_;
  }

private:
  size_t j_;

  Frame p_out_;
};
}  // namespace kdl

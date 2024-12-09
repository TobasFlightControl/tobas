#pragma once

#include "./chain_solver_i.hpp"
#include "./frame_vel.hpp"
#include "./jntarray.hpp"

namespace kdl
{
/**
 * Implementation of a recursive forward position and velocity
 * kinematics algorithm to calculate the position and velocity
 * transformation from joint space to Cartesian space of a general
 * kinematic chain (kdl::Chain).
 */
class ChainFkSolverVel_recursive : public ChainSolverI
{
  using super = ChainSolverI;

public:
  explicit ChainFkSolverVel_recursive(const Chain& chain);

  /* Calculate forward position and velocity kinematics, from joint coordinates to cartesian coordinates. */
  int JntToCart(const JntArray& q_in, const JntArray& qd_in, int seg_nr = -1);

  const FrameVel& getFrameVel() const
  {
    return p_out_;
  }

private:
  size_t j_;
  double qj_, qdj_;

  FrameVel p_out_;
};
}  // namespace kdl

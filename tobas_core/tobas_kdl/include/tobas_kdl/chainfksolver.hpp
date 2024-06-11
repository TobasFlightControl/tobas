#pragma once

#include "./chainsolveri.hpp"
#include "./framevel.hpp"
#include "./jntarray.hpp"

namespace tobas_kdl
{
/**
 * \brief This <strong>abstract</strong> class encapsulates a
 * solver for the forward position kinematics for a tobas_kdl::Chain.
 */
class ChainFkSolverPos : public ChainSolverI
{
  using super = ChainSolverI;

public:
  explicit ChainFkSolverPos(const Chain& chain) : super(chain)
  {
  }

  /**
   * Calculate forward position kinematics for a tobas_kdl::Chain,
   * from joint coordinates to cartesian pose.
   */
  virtual int JntToCart(const JntArray& q_in, int seg_nr = -1) = 0;

  const Frame& getFrame() const
  {
    return p_out_;
  }

protected:
  Frame p_out_;
};

/**
 * \brief This <strong>abstract</strong> class encapsulates a solver
 * for the forward velocity kinematics for a tobas_kdl::Chain.
 */
class ChainFkSolverVel : public ChainSolverI
{
  using super = ChainSolverI;

public:
  explicit ChainFkSolverVel(const Chain& chain) : super(chain)
  {
  }

  /**
   * Calculate forward position and velocity kinematics, from
   * joint coordinates to cartesian coordinates.
   */
  virtual int JntToCart(const JntArray& q_in, const JntArray& qd_in, int seg_nr = -1) = 0;

  const FrameVel& getFrameVel() const
  {
    return p_out_;
  }

protected:
  FrameVel p_out_;
};
}  // end of namespace tobas_kdl

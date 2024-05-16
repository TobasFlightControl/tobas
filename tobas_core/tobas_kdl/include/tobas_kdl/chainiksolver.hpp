#pragma once

#include "./chainsolveri.hpp"
#include "./frames.hpp"
#include "./framevel.hpp"
#include "./jntarray.hpp"
#include "./jntarrayvel.hpp"

namespace tobas_kdl
{
/**
 * \brief This <strong>abstract</strong> class encapsulates the inverse
 * position solver for a tobas_kdl::Chain.
 */
class ChainIkSolverPos : public ChainSolverI
{
  using super = ChainSolverI;

public:
  explicit ChainIkSolverPos(const Chain& chain) : super(chain)
  {
  }

  /**
   * Calculate inverse position kinematics, from cartesian
   * coordinates to joint coordinates.
   *
   * @param q_init initial guess of the joint coordinates
   * @param p_in input cartesian coordinates
   * @param q_out output joint coordinates
   *
   * @return if < 0 something went wrong
   */
  virtual int CartToJnt(const JntArray& q_init, const Frame& p_in) = 0;

  const JntArray& getPositions() const
  {
    return q_out_;
  }

protected:
  JntArray q_out_;
};

/**
 * \brief This <strong>abstract</strong> class encapsulates the inverse
 * velocity solver for a tobas_kdl::Chain.
 */
class ChainIkSolverVel : public ChainSolverI
{
  using super = ChainSolverI;

public:
  explicit ChainIkSolverVel(const Chain& chain) : super(chain)
  {
  }

  /**
   * Calculate inverse velocity kinematics, from joint positions
   * and cartesian velocity to joint velocities.
   *
   * @param q_in input joint positions
   * @param v_in input cartesian velocity
   * @param qdot_out output joint velocities
   *
   * @return if < 0 something went wrong
   */
  virtual int CartToJnt(const JntArray& q_in, const Vector& v_in) = 0;

  /**
   * Calculate inverse velocity kinematics, from joint positions
   * and cartesian velocity to joint velocities.
   *
   * @param q_in input joint positions
   * @param v_in input cartesian velocity
   * @param qdot_out output joint velocities
   *
   * @return if < 0 something went wrong
   */
  virtual int CartToJnt(const JntArray& q_in, const Twist& v_in) = 0;

  const JntArray& getVelocities() const
  {
    return qd_out_;
  }

protected:
  JntArray qd_out_;
};

/**
 * \brief This abstract class encapsulates the inverse acceleration solver for a tobas_kdl::Tree.
 */
class ChainIkSolverAcc : public ChainSolverI
{
  using super = ChainSolverI;

public:
  explicit ChainIkSolverAcc(const Chain& chain) : super(chain)
  {
  }

  /**
   * Calculate inverse acceleration kinematics, from joint positions
   * and cartesian velocities to joint velocities.
   *
   * @param q_in input joint positions
   * @param qd_in input joint velocities
   * @param a_in input cartesian acceleration
   *
   * @return if < 0 something went wrong.
   */
  virtual int CartToJnt(const JntArray& q_in, const JntArray& qd_in, const Vector& a_in) = 0;

  /**
   * Calculate inverse acceleration kinematics, from joint positions
   * and cartesian velocities to joint velocities.
   *
   * @param q_in input joint positions
   * @param qd_in input joint velocities
   * @param a_in input cartesian acceleration
   *
   * @return if < 0 something went wrong.
   */
  virtual int CartToJnt(const JntArray& q_in, const JntArray& qd_in, const Accel& a_in) = 0;

  const JntArray& getAccelerations() const
  {
    return qdd_out_;
  }

protected:
  JntArray qdd_out_;
};
}  // end of namespace tobas_kdl

#pragma once

#include "./treesolveri.hpp"
#include "./frames.hpp"
#include "./jntarray.hpp"
#include "./jacobian.hpp"

namespace KDL
{
static constexpr double kDefaultWeightTS = 1.;
static constexpr double kDefaultWeightJS = 1e-3;  // TODO: 関節ごとに支持重量でスケーリング

/**
 * \brief This abstract class encapsulates the inverse position solver for a KDL::Tree.
 */
class TreeIkSolverPos : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeIkSolverPos(const Tree& tree) : super(tree)
  {
  }

  /**
   * Calculate inverse position kinematics, from cartesian
   * coordinates to joint coordinates.
   *
   * @param q_init initial guess of the joint coordinates
   * @param p_in input cartesian coordinates
   *
   * @return if < 0 something went wrong.
   */
  virtual int CartToJnt(const JntArray& q_init, const FrameMap& p_in) = 0;

  /* Get resulting joint positions. */
  const JntArray& getPositions() const
  {
    return q_out_;
  }

protected:
  JntArray q_out_;
};

/**
 * \brief This abstract class encapsulates the inverse velocity solver for a KDL::Tree.
 */
class TreeIkSolverVel : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeIkSolverVel(const Tree& tree) : super(tree)
  {
  }

  /**
   * Calculate inverse velocity kinematics, from joint positions
   * and cartesian velocities to joint velocities.
   *
   * @param q_in input joint positions
   * @param v_in input cartesian velocity
   *
   * @return if < 0 something went wrong.
   */
  virtual int CartToJnt(const JntArray& q_in, const TwistMap& v_in) = 0;

  /* Get resulting joint velocities. */
  const JntArray& getVelocities() const
  {
    return qd_out_;
  }

protected:
  JntArray qd_out_;
};

/**
 * \brief This abstract class encapsulates the inverse acceleration solver for a KDL::Tree.
 */
class TreeIkSolverAcc : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeIkSolverAcc(const Tree& tree) : super(tree)
  {
  }

  /**
   * Calculate inverse acceleration kinematics, from joint positions
   * and cartesian velocities to joint velocities.
   *
   * @param q_in input joint positions
   * @param qd_in input joint velocities
   * @param acc_in input cartesian acceleration
   *
   * @return if < 0 something went wrong.
   */
  virtual int CartToJnt(const JntArray& q_in, const JntArray& qd_in, const AccelMap& acc_in) = 0;

  /* Get resulting joint accelerations. */
  const JntArray& getAccelerations() const
  {
    return qdd_out_;
  }

protected:
  JntArray qdd_out_;
};
}  // namespace KDL

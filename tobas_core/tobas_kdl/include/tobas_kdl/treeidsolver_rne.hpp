#pragma once

#include "./treeidsolver.hpp"
#include "./utilities/constants.hpp"

namespace KDL
{
/**
 * \brief Recursive newton euler inverse dynamics solver for kinematic trees.
 *
 * It calculates the torques for the joints, given the motion of
 * the joints (q,qdot,qdotdot), external forces on the segments
 * (expressed in the segments reference frame) and the dynamical
 * parameters of the segments.
 *
 * This is an extension of the inverse dynamic solver for kinematic chains,
 * \see ChainIdSolver_RNE. The main difference is the use of STL maps
 * instead of vectors to represent external wrenches (as well as internal
 * variables exploited during the recursion).
 */
class TreeIdSolver_RNE : public TreeIdSolver
{
  using super = TreeIdSolver;

public:
  /**
   * Constructor for the solver, it will allocate all the necessary memory
   * \param tree The kinematic tree to calculate the inverse dynamics for, an internal reference
   * will be stored. \param grav The gravity vector to use during the calculation.
   */
  explicit TreeIdSolver_RNE(const Tree& tree, const Vector& grav = Vector(0, 0, -kDefaultGravity));

  void updateInternalDataStructures() override;

  /**
   * Function to calculate from Cartesian forces to joint torques.
   * Input parameters;
   * \param q The current joint positions
   * \param q_dot The current joint velocities
   * \param q_dotdot The current joint accelerations
   * \param f_ext The external forces (no gravity) on the segments
   */
  int CartToJnt(
    const JntArray& q,
    const JntArray& q_dot,
    const JntArray& q_dotdot,
    const WrenchMap& f_ext = WrenchMap()) override;

private:
  const Accel ag_;

  TwistMap v_;
  AccelMap a_;
  WrenchMap f_;

  /* One recursion step */
  void rneStep(
    const SegmentMap::const_iterator& segment,
    const JntArray& q,
    const JntArray& q_dot,
    const JntArray& q_dotdot,
    const WrenchMap& f_ext);
};
}  // namespace KDL

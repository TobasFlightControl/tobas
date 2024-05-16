#pragma once

#include "./chainsolveri.hpp"
#include "./frames.hpp"
#include "./jacobian.hpp"
#include "./jntarray.hpp"

namespace tobas_kdl
{
/**
 * @brief  Class to calculate the jacobian of a general
 * tobas_kdl::Chain, it is used by other solvers.
 */
class ChainJntToJacSolver : public ChainSolverI
{
  using super = ChainSolverI;

public:
  explicit ChainJntToJacSolver(const Chain& chain);

  virtual void updateInternalDataStructures() override;

  /**
   * Calculate the jacobian expressed in the base frame of the
   * chain, with reference point at the end effector of the
   * *chain. The algorithm is similar to the one used in
   * tobas_kdl::ChainFkSolverVel_recursive
   *
   * @param q_in input joint positions
   * @param jac output jacobian
   * @param seg_nr The final segment to compute
   * @return success/error code
   */
  virtual int JntToJac(const JntArray& q_in, int seg_nr = -1);

  /**
   * @param locked_joints new values for locked joints
   * @return success/error code
   */
  bool setLockedJoints(const std::vector<bool> locked_joints);

  const Jacobian& getJacobian() const
  {
    return jac_out_;
  }

private:
  std::vector<bool> locked_joints_;
  Frame total_;
  Jacobian jac_out_;
};
}  // namespace tobas_kdl

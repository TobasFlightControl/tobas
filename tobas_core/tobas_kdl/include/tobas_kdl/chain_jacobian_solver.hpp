#pragma once

#include "./chain_solver_i.hpp"
#include "./frames.hpp"
#include "./jacobian.hpp"
#include "./jntarray.hpp"

namespace kdl
{
/**
 * @brief  Class to calculate the jacobian of a general kdl::Chain, it is used by other solvers.
 */
class ChainJacobianSolver : public ChainSolverI
{
  using super = ChainSolverI;

public:
  explicit ChainJacobianSolver(const Chain& chain);

  virtual bool updateInternalDataStructures() override;

  /**
   * Calculate the jacobian expressed in the base frame of the
   * chain, with reference point at the end effector of the
   * *chain. The algorithm is similar to the one used in
   * kdl::ChainFkSolverVel_recursive
   *
   * @param q_in input joint positions
   * @param jac output jacobian
   * @param seg_nr The final segment to compute
   * @return success/error code
   */
  int JntToJac(const JntArray& q_in, int seg_nr = -1);

  /**
   * @param locked_joints new values for locked joints
   * @return success/error code
   */
  bool setLockedJoints(const std::vector<bool> locked_joints);

  const Jacobian& getJacobian() const
  {
    return J_out_;
  }

private:
  std::vector<bool> locked_joints_;
  Jacobian J_out_;
  Frame T_tmp_;
  size_t j_, k_;

  void resize();
};
}  // namespace kdl

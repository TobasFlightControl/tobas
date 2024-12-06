#pragma once

#include "./chain_ik_solver.hpp"
#include "./chain_jacobian_solver.hpp"
#include "./chain_jac_acc_solver.hpp"

namespace kdl
{
/* 分解加速度制御 */
class ChainIkSolverAcc_RAC : public ChainIkSolverAcc
{
  using super = ChainIkSolverAcc;

public:
  explicit ChainIkSolverAcc_RAC(const Chain& chain);

  void updateInternalDataStructures() override;

  int CartToJnt(const JntArray& q, const JntArray& qd, const Vector& a) override;
  int CartToJnt(const JntArray& q, const JntArray& qd, const Accel& a) override;

private:
  ChainJacobianSolver jnt2jac_;
  ChainJacAccSolver jnt2jdqd_;
};
}  // namespace kdl

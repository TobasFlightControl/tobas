#pragma once

#include "./chain_ik_solver.hpp"
#include "./chain_jacobian_solver.hpp"

namespace kdl
{
/* 公式のChainIkSolverVel_pinvを並進速度のみの場合にも対応させたもの */
class ChainIkSolverVel_pinv : public ChainIkSolverVel
{
  using super = ChainIkSolverVel;

public:
  explicit ChainIkSolverVel_pinv(const Chain& chain);

  void updateInternalDataStructures() override;

  int CartToJnt(const JntArray& q, const Vector& v) override;
  int CartToJnt(const JntArray& q, const Twist& v) override;

private:
  ChainJacobianSolver jnt2jac_;
};
}  // namespace kdl

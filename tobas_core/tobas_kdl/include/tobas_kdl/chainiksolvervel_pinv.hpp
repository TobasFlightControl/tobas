#pragma once

#include "./chainiksolver.hpp"
#include "./chainjnttojacsolver.hpp"

namespace KDL
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
  ChainJntToJacSolver jnt2jac_;
};
}  // namespace KDL

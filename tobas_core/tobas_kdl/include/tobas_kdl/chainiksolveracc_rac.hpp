#pragma once

#include "./chainiksolver.hpp"
#include "./chainjnttojacsolver.hpp"
#include "./chainjnttojacaccsolver.hpp"

namespace tobas_kdl
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
  ChainJntToJacSolver jnt2jac_;
  ChainJntToJacAccSolver jnt2jdqd_;
};
}  // namespace tobas_kdl

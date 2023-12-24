#pragma once

#include "./chainsolveri.hpp"
#include "./chainjnttojacsolver.hpp"
#include "./chainjnttojacaccsolver.hpp"

namespace KDL
{
/* 分解加速度制御 */
class ChainIkSolverAcc_RAC : public ChainSolverI
{
  using super = ChainSolverI;

public:
  explicit ChainIkSolverAcc_RAC(const Chain& chain);

  void updateInternalDataStructures() override;

  /* 並進のみの場合 */
  int CartToJnt(const JntArray& q, const JntArray& qd, const Vector& acc_ref);

  /* 並進と回転を含む場合 */
  int CartToJnt(const JntArray& q, const JntArray& qd, const Accel& acc_ref);

  const JntArray& getVelocities() const
  {
    return qdd_out_;
  }

private:
  ChainJntToJacSolver jnt2jac_;
  ChainJntToJacAccSolver jnt2jdqd_;

  JntArray qdd_out_;
};
}  // namespace KDL

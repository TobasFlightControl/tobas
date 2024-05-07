#pragma once

#include "./chainsolveri.hpp"
#include "./jntspaceinertiamatrix.hpp"

namespace KDL
{
/* ベースは公式のChainDynParamで，重力加速度を呼び出し時に与えるようにしたもの */
class ChainJntSpaceInertiaSolver : public ChainSolverI
{
  using super = ChainSolverI;

public:
  explicit ChainJntSpaceInertiaSolver(const Chain& chain);

  void updateInternalDataStructures() override;

  int JntToMass(const JntArray& q);
  inline const JntSpaceInertiaMatrix& getMass() const;

private:
  std::vector<RigidBodyInertia> I_;
  std::vector<Frame> X_;
  std::vector<SegmentJacobian> S_;

  JntSpaceInertiaMatrix H_out_;
};

inline const JntSpaceInertiaMatrix& ChainJntSpaceInertiaSolver::getMass() const
{
  return H_out_;
}
}  // namespace KDL

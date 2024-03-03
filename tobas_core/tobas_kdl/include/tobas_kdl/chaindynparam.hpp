#pragma once

#include "./chainsolveri.hpp"
#include "./jntarray.hpp"
#include "./jntspaceinertiamatrix.hpp"
#include "./chainidsolver_rne.hpp"

namespace KDL
{
/* ベースは公式のChainDynParamで，重力加速度を呼び出し時に与えるようにしたもの */
class ChainDynParam : public ChainSolverI
{
  using super = ChainSolverI;

public:
  explicit ChainDynParam(const Chain& chain);

  void updateInternalDataStructures() override;

  int JntToMass(const JntArray& q);
  int JntToCoriolis(const JntArray& q, const JntArray& qd);
  int JntToGravity(const JntArray& q, const Vector& grav);

  inline const JntSpaceInertiaMatrix& getJntInertia() const;
  inline const JntArray& getCoriolisEffort() const;
  inline const JntArray& getGravityEffort() const;

private:
  ChainIdSolver_RNE rne_coriolis_;
  ChainIdSolver_RNE rne_gravity_;

  JntArray zero_jntarray_;
  Wrenches zero_wrenches_;
  const Vector zero_vector_ = Vector::Zero();
  std::vector<RigidBodyInertia> I_;
  std::vector<Frame> X_;
  std::vector<SegmentJacobian> S_;

  JntSpaceInertiaMatrix H_out_;
};

inline const JntSpaceInertiaMatrix& ChainDynParam::getJntInertia() const
{
  return H_out_;
}

inline const JntArray& ChainDynParam::getCoriolisEffort() const
{
  return rne_coriolis_.getEfforts();
}

inline const JntArray& ChainDynParam::getGravityEffort() const
{
  return rne_gravity_.getEfforts();
}
}  // namespace KDL

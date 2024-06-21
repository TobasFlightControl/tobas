#pragma once

#include "./chainsolveri.hpp"
#include "./frames.hpp"
#include "./jntarray.hpp"

namespace kdl
{
/* ベースは公式のChainIdSolver_RNEで，重力加速度を呼び出し時に与えるようにしたもの */
class ChainIdSolver_RNE : public ChainSolverI
{
  using super = ChainSolverI;

public:
  explicit ChainIdSolver_RNE(const Chain& chain);

  void updateInternalDataStructures() override;

  int CartToJnt(
    const JntArray& q,
    const JntArray& qd,
    const JntArray& qdd,
    const Wrenches& forces,  // 各フレームにかかる力を各フレームから見たもの
    const Vector& grav);

  int CartToJnt(
    const JntArray& q,
    const JntArray& qd,
    const JntArray& qdd,
    const Wrench& f_ee,  // EEにかかる力をベースから見たもの
    const Vector& grav);

  inline const JntArray& getEfforts() const;

private:
  std::vector<Frame> X_;
  std::vector<SegmentJacobian> S_;
  std::vector<Twist> v_;
  std::vector<Accel> a_;
  std::vector<Wrench> f_;

  int j_;
  double qj_, qdj_, qddj_;

  JntArray effort_out_;
};

inline const JntArray& ChainIdSolver_RNE::getEfforts() const
{
  return effort_out_;
}
}  // namespace kdl

#pragma once

#include "./chainsolveri.hpp"
#include "./jntarray.hpp"

namespace kdl
{
/**
 * @brief xdd = J qd + Jd qd におけるJd qdの項を計算する．
 * qdd = 0, grav = 0としてRNEの順伝搬を行うことでJd qdが求められる．
 * cf. chainidsolver_rne.cpp
 */
class ChainJntToJacAccSolver : public ChainSolverI
{
  using super = ChainSolverI;

public:
  explicit ChainJntToJacAccSolver(const Chain& chain);

  void updateInternalDataStructures() override;

  int JntToCart(const JntArray& q, const JntArray& qd);

  const Accel& getJdqd() const
  {
    return Jdqd_out_;
  }

private:
  std::vector<Frame> X_;
  std::vector<Twist> v_;
  std::vector<Accel> a_;
  Accel Jdqd_out_;
  size_t j_;
  double qj_, qdj_;
};
}  // namespace kdl

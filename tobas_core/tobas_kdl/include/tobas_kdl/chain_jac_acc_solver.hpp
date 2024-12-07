#pragma once

#include "./chain_solver_i.hpp"
#include "./jntarray.hpp"

namespace kdl
{
/**
 * @brief xdd = J qd + Jd qd におけるJd qdの項を計算する．
 * qdd = 0, grav = 0としてRNEの順伝搬を行うことでJd qdが求められる．
 * cf. chain_id_solver_rne.cpp
 */
class ChainJacAccSolver : public ChainSolverI
{
  using super = ChainSolverI;

public:
  explicit ChainJacAccSolver(const Chain& chain);

  bool updateInternalDataStructures() override;

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

  void resize();
};
}  // namespace kdl

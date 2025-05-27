#pragma once

#include "./chain.hpp"
#include "./chain_solver_i.hpp"
#include "./jntarray.hpp"
#include "./rigid_body_inertia.hpp"

namespace kdl
{
class ChainInertiaSolver : public ChainSolverI
{
  using super = ChainSolverI;

public:
  explicit ChainInertiaSolver(const Chain& chain);

  bool updateInternalDataStructures() override;

  int JntToCart(const JntArray& q);

  inline const RigidBodyInertia& getInertia() const;

private:
  std::vector<RigidBodyInertia> I_;
  std::vector<Frame> X_;
  RigidBodyInertia I_out_;
  size_t j_;

  void resize();
};

inline const RigidBodyInertia& ChainInertiaSolver::getInertia() const
{
  return I_out_;
}
}  // namespace kdl

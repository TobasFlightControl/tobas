#pragma once

#include "./chain_solver_i.hpp"
#include "./chain.hpp"
#include "./jntarray.hpp"
#include "./rigid_body_inertia.hpp"

namespace kdl
{
class ChainInertiaSolver : public ChainSolverI
{
  using super = ChainSolverI;

public:
  explicit ChainInertiaSolver(const Chain& chain);

  void updateInternalDataStructures() override;

  int JntToCart(const JntArray& q);

  const RigidBodyInertia& getInertia() const
  {
    return I_out_;
  }

private:
  std::vector<RigidBodyInertia> I_;
  std::vector<Frame> X_;
  RigidBodyInertia I_out_;
  size_t j_;
};
}  // namespace kdl

#pragma once

#include "./chainsolveri.hpp"
#include "./chain.hpp"
#include "./jntarray.hpp"
#include "./rigidbodyinertia.hpp"

namespace tobas_kdl
{
class ChainJntToInertiaSolver : public ChainSolverI
{
  using super = ChainSolverI;

public:
  explicit ChainJntToInertiaSolver(const Chain& chain);

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
};
}  // namespace tobas_kdl

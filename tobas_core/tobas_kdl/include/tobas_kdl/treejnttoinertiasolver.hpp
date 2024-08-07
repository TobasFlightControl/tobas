#pragma once

#include "./treesolveri.hpp"
#include "./jntarray.hpp"
#include "./rigidbodyinertia.hpp"

namespace kdl
{
class TreeJntToInertiaSolver : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeJntToInertiaSolver(const Tree& tree);

  void updateInternalDataStructures() override;

  /* ルートリンク周りの質量特性を計算する． */
  int JntToCart(const JntArray& q);

  inline const RigidBodyInertia& getInertia() const
  {
    return I_.at(tree_.getRootSegment()->first);
  }

private:
  std::map<std::string, Frame> X_;
  std::map<std::string, RigidBodyInertia> I_;

  void step(const SegmentMap::const_iterator& cur_it, const JntArray& q);
};
}  // namespace kdl
